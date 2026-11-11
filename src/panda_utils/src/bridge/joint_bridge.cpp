#include "panda_utils/bridge/joint_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace panda_bridge {

namespace {

using Mode = PipelineTypes::Mode;
using FinishReason = PipelineTypes::FinishReason;
using PipelineState = PipelineTypes::State;

std::vector<double> to_vector(const JointArray &values) { return std::vector<double>(values.begin(), values.end()); }

}  // namespace

template <JointInterface I>
JointBridge<I>::JointBridge(const rclcpp::NodeOptions &options) : BridgeBase(Traits::kNodeName, options) {
  declare_parameter<std::string>("mode", "external");
  declare_parameter<std::string>("auto_preset", "");
  // Torque samples are held between arrivals, not fitted: a short delay is enough
  declare_parameter<double>("playout_delay_ms", I == JointInterface::kTorques ? 1.0 : 5.0);
  declare_parameter<double>("stall_timeout_ms", 20.0);
  declare_parameter<double>("command_timeout_ms", 0.0);
  declare_parameter<bool>("clamp", true);
  // Kinematic limits of the commands and of the internal motions, as fractions of the FER limits
  declare_parameter<double>("safe_joint_speed_factor", 0.9);
  declare_parameter<double>("safe_joint_acceleration_factor", 0.8);
  declare_parameter<double>("safe_joint_jerk_factor", 0.5);
  declare_parameter<double>("joint_limit_margin", 0.01);
  if constexpr (I == JointInterface::kPositions) {
    // Deprecated: same as mode internal with auto_preset quintic_joint1
    declare_parameter<bool>("use_demo", false);
    declare_parameter<double>("max_jump", 0.2);
  }
  if constexpr (I == JointInterface::kTorques) {
    // Effort limits and torque rate limit as fractions of the FER ones; joint impedance gains of
    // the internal motions, of the hold and (damping) of the stops
    const TorqueSettings defaults = fer_torque_settings();
    declare_parameter<double>("safe_torque_factor", 0.9);
    declare_parameter<double>("torque_rate_factor", 0.5);
    declare_parameter<std::vector<double>>("joint_stiffness", to_vector(defaults.stiffness));
    declare_parameter<std::vector<double>>("joint_damping", to_vector(defaults.damping));
  } else {
    declare_parameter<std::string>("controller_mode", "joint_impedance");
  }
}

template <JointInterface I>
JointBridge<I>::~JointBridge() {
  teardown();
}

template <JointInterface I>
bool JointBridge<I>::read_bridge_parameters(PipelineTypes::Config &config, std::string &error) {
  std::string mode = get_parameter("mode").as_string();
  std::string auto_preset = get_parameter("auto_preset").as_string();
  if constexpr (I == JointInterface::kPositions) {
    if (get_parameter("use_demo").as_bool()) {
      RCLCPP_WARN(get_logger(), "use_demo is deprecated: use mode internal and auto_preset quintic_joint1");
      mode = "internal";
      if (auto_preset.empty()) {
        auto_preset = "quintic_joint1";
      }
    }
  }
  if (mode == "external") {
    config.mode = Mode::kExternal;
  } else if (mode == "internal") {
    config.mode = Mode::kInternal;
  } else {
    error = "mode must be external or internal, not '" + mode + "'";
    return false;
  }

  if constexpr (I != JointInterface::kTorques) {
    const std::string controller_mode = get_parameter("controller_mode").as_string();
    if (controller_mode == "joint_impedance") {
      controller_mode_ = franka::ControllerMode::kJointImpedance;
    } else if (controller_mode == "cartesian_impedance") {
      controller_mode_ = franka::ControllerMode::kCartesianImpedance;
    } else {
      error = "controller_mode must be joint_impedance or cartesian_impedance";
      return false;
    }
  }

  const double playout_delay_ms = get_parameter("playout_delay_ms").as_double();
  const double stall_timeout_ms = get_parameter("stall_timeout_ms").as_double();
  const double command_timeout_ms = get_parameter("command_timeout_ms").as_double();
  const double speed = get_parameter("safe_joint_speed_factor").as_double();
  const double acceleration = get_parameter("safe_joint_acceleration_factor").as_double();
  const double jerk = get_parameter("safe_joint_jerk_factor").as_double();
  config.clamp = get_parameter("clamp").as_bool();
  config.position_margin = get_parameter("joint_limit_margin").as_double();
  if constexpr (I == JointInterface::kPositions) {
    config.max_jump = get_parameter("max_jump").as_double();
  }

  auto factor_ok = [](double f) { return f > 0.0 && f <= 1.0; };
  if (!(playout_delay_ms >= 0.0 && playout_delay_ms <= 1000.0) || !(stall_timeout_ms > 0.0) ||
      !(command_timeout_ms >= 0.0) || !(config.max_jump > 0.0) || !(config.position_margin >= 0.0) ||
      !factor_ok(speed) || !factor_ok(acceleration) || !factor_ok(jerk)) {
    error = "playout_delay_ms in [0, 1000], stall_timeout_ms > 0, command_timeout_ms >= 0, max_jump > 0, "
            "joint_limit_margin >= 0 and safe_joint_*_factor in (0, 1] are required";
    return false;
  }
  config.playout_delay_ns = static_cast<int64_t>(playout_delay_ms * 1e6);
  config.stall_timeout_ns = static_cast<int64_t>(stall_timeout_ms * 1e6);
  config.command_timeout_ns = static_cast<int64_t>(command_timeout_ms * 1e6);
  config.limits = fer_joint_limits(speed, acceleration, jerk);
  if constexpr (I == JointInterface::kTorques) {
    if (!read_torque_parameters(config.torque, error)) {
      return false;
    }
  }
  auto_preset_ = auto_preset;
  return true;
}

template <JointInterface I>
bool JointBridge<I>::read_torque_parameters(TorqueSettings &settings, std::string &error) {
  const double torque_factor = get_parameter("safe_torque_factor").as_double();
  const double rate_factor = get_parameter("torque_rate_factor").as_double();
  const std::vector<double> stiffness = get_parameter("joint_stiffness").as_double_array();
  const std::vector<double> damping = get_parameter("joint_damping").as_double_array();
  if (!(torque_factor > 0.0 && torque_factor <= 1.0) || !(rate_factor > 0.0 && rate_factor <= 1.0)) {
    error = "safe_torque_factor and torque_rate_factor must be in (0, 1]";
    return false;
  }
  if (stiffness.size() != kNumJoints || damping.size() != kNumJoints) {
    error = "joint_stiffness and joint_damping need 7 values";
    return false;
  }
  settings = fer_torque_settings(torque_factor, rate_factor);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!(stiffness[i] >= 0.0) || !(damping[i] >= 0.0)) {
      error = "joint_stiffness and joint_damping must be >= 0";
      return false;
    }
    settings.stiffness[i] = stiffness[i];
    settings.damping[i] = damping[i];
  }
  return true;
}

template <JointInterface I>
bool JointBridge<I>::configure_bridge() {
  std::string error;
  PipelineTypes::Config config;
  if (!read_bridge_parameters(config, error)) {
    RCLCPP_ERROR(get_logger(), "Invalid parameters: %s", error.c_str());
    return false;
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = command_group();
  // Best effort with a deep history: accepts reliable and best-effort publishers, keeps bursts
  const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();
  command_sub_ = create_subscription<CommandMsg>(
      Traits::kCommandTopic, command_qos, [this](const typename CommandMsg::SharedPtr msg) { on_command(msg); },
      options);

  action_server_ = rclcpp_action::create_server<JointTraj>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "/panda/joint_traj",
      [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const JointTraj::Goal> goal) {
        return on_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandle> handle) { return on_cancel(handle); },
      [this](const std::shared_ptr<GoalHandle> handle) { on_accepted(handle); },
      rcl_action_server_get_default_options(), services_group());

  preset_srv_ = create_service<uclv_panda_interfaces::srv::RunPreset>(
      "/panda/run_preset",
      [this](const std::shared_ptr<uclv_panda_interfaces::srv::RunPreset::Request> request,
             std::shared_ptr<uclv_panda_interfaces::srv::RunPreset::Response> response) {
        response->accepted = start_preset(request->preset, request->duration, request->amplitude, response->message);
      },
      rclcpp::ServicesQoS(), services_group());

  stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/panda/stop_motion",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!accepts_motions()) {
          response->success = false;
          response->message = "no internal motion to stop";
          return;
        }
        JointMotionRequest cancel;
        cancel.kind = JointMotionRequest::Kind::kCancel;
        cancel.id = 0;
        response->success = push_request(cancel);
        response->message = response->success ? "stopping" : "request queue full";
      },
      rclcpp::ServicesQoS(), services_group());

  goal_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { update_goals(); }, services_group());
  return true;
}

template <JointInterface I>
void JointBridge<I>::cleanup_bridge() {
  goal_timer_.reset();
  command_sub_.reset();
  preset_srv_.reset();
  stop_srv_.reset();
  goals_.abort_all();
  action_server_.reset();
}

template <JointInterface I>
bool JointBridge<I>::prepare_control(std::string &error) {
  PipelineTypes::Config config;
  if (!read_bridge_parameters(config, error)) {
    return false;
  }
  config_ = config;
  pipeline_.configure(config_);
  // Inputs left over from a previous run must not reach the new one
  typename Pipeline::Sample sample;
  while (samples_.pop(sample)) {
  }
  JointMotionRequest request;
  while (requests_.pop(request)) {
  }
  snapshot_box_.set(Snapshot{});
  rt_previous_ = Snapshot{};
  non_finite_command_ = false;
  stop_requested_ = false;
  internal_mode_ = config_.mode == Mode::kInternal;
  RCLCPP_INFO(get_logger(), "Control run in %s mode", internal_mode_ ? "internal" : "external");
  return true;
}

template <JointInterface I>
void JointBridge<I>::on_control_started() {
  if (!internal_mode_ || auto_preset_.empty()) {
    return;
  }
  for (int i = 0; i < 100 && !snapshot().valid; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::string message;
  if (start_preset(auto_preset_, 0.0, 0.0, message)) {
    RCLCPP_INFO(get_logger(), "auto_preset: %s", message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "auto_preset not started: %s", message.c_str());
  }
}

template <JointInterface I>
void JointBridge<I>::run_control(RobotBackend &robot) {
  rt_model_ = robot.model();
  accepting_samples_ = config_.mode == Mode::kExternal;
  const auto callback = [this](const franka::RobotState &state, franka::Duration period) {
    return rt_update(state, period);
  };
  try {
    if constexpr (I == JointInterface::kPositions) {
      robot.control_joint_positions(callback, controller_mode_, params().limit_rate, params().lpf_cutoff_frequency);
    } else if constexpr (I == JointInterface::kVelocities) {
      robot.control_joint_velocities(callback, controller_mode_, params().limit_rate, params().lpf_cutoff_frequency);
    } else {
      robot.control_torques(callback, params().limit_rate, params().lpf_cutoff_frequency);
    }
  } catch (...) {
    accepting_samples_ = false;
    throw;
  }
  accepting_samples_ = false;
}

template <JointInterface I>
JointFeedback JointBridge<I>::rt_feedback(const franka::RobotState &state) const {
  JointFeedback robot;
  robot.q = state.q;
  robot.dq = state.dq;
  robot.q_d = state.q_d;
  robot.dq_d = state.dq_d;
  robot.ddq_d = state.ddq_d;
  robot.tau_d = state.tau_J_d;
  if constexpr (I == JointInterface::kTorques) {
    // Without a model (fake robot) the Coriolis torque stays zero
    if (rt_model_ != nullptr) {
      robot.coriolis = rt_model_->coriolis(state);
    }
  }
  return robot;
}

template <JointInterface I>
typename JointBridge<I>::FrankaCommand JointBridge<I>::rt_update(const franka::RobotState &state,
                                                                 franka::Duration period) {
  rt_cycle_begin(state, period);
  const int64_t wall = wall_clock_ns();
  const int64_t robot_time = static_cast<int64_t>(state.time.toMSec()) * kCycleNs;
  const uint64_t elapsed = period.toMSec();
  const JointFeedback robot = rt_feedback(state);
  if (elapsed == 0) {
    // First cycle of the control run: start from the state reported by the robot
    playback_clock_.reset(wall, robot_time);
    pipeline_.reset(robot, wall);
  } else {
    // Playback on the robot clock, aligned to the clock of the stamps
    const int64_t now = playback_clock_.update(wall, robot_time);
    pipeline_.set_feedback(robot);
    typename Pipeline::Sample sample;
    while (samples_.pop(sample)) {
      pipeline_.push_sample(sample);
    }
    JointMotionRequest request;
    while (requests_.pop(request)) {
      pipeline_.request(request);
    }
    if (stop_requested_.load(std::memory_order_acquire)) {
      pipeline_.request_finish(FinishReason::kRequested);
    }
    if (below_critical_height(state)) {
      pipeline_.request_finish(FinishReason::kSafety);
    }
    // After lost packets the command must match the elapsed time
    const uint64_t cycles = std::min<uint64_t>(elapsed, 10);
    for (uint64_t k = cycles; k > 0; --k) {
      pipeline_.step(now - static_cast<int64_t>(k - 1) * kCycleNs);
    }
  }
  rt_publish_snapshot();

  FrankaCommand command(pipeline_.command());
  bool finished = pipeline_.state() == PipelineState::kFinished;
  if (!all_finite(pipeline_.command())) {
    // Never expected: keep the robot where it is
    (void)events_.push(RtEvent{kNonFiniteCommand, 0, 0.0});
    non_finite_command_ = true;
    if constexpr (I == JointInterface::kPositions) {
      command = FrankaCommand(state.q_d);
    } else {
      command = FrankaCommand(JointArray{});
    }
    finished = true;
  }
  rt_cycle_end();
  return finished ? franka::MotionFinished(command) : command;
}

template <JointInterface I>
void JointBridge<I>::rt_publish_snapshot() {
  Snapshot current;
  current.valid = true;
  current.state = pipeline_.state();
  current.counters = pipeline_.counters();
  current.motion = pipeline_.motion_status();
  current.position = pipeline_.position();
  current.tracker_errors = pipeline_.tracker_errors();
  push_pipeline_events(events_, rt_previous_, current, ns_to_s(config_.stall_timeout_ns) * 1e3);
  rt_previous_ = current;
  (void)snapshot_box_.try_set(current);
}

template <JointInterface I>
std::string JointBridge<I>::finish_reason() const {
  if (non_finite_command_) {
    return std::string("non-finite ") + Traits::kQuantity + " command";
  }
  switch (pipeline_.finish_reason()) {
    case FinishReason::kRequested:
      return "";
    case FinishReason::kCommandTimeout:
      return "no commands for " + std::to_string(config_.command_timeout_ns / 1'000'000) + " ms (command_timeout_ms)";
    case FinishReason::kSafety:
      return "end effector below the critical height of " + std::to_string(params().critical_height) + " m";
    case FinishReason::kLimits:
      return std::string(Traits::kQuantity) + " beyond the limits (clamp disabled)";
    case FinishReason::kNone:
    default:
      return "control loop finished";
  }
}

template <JointInterface I>
bool JointBridge<I>::command_values(const CommandMsg &msg, JointArray &values) {
  if constexpr (I == JointInterface::kPositions) {
    values = msg.positions;
    return true;
  } else {
    const std::vector<double> &source = I == JointInterface::kVelocities ? msg.velocity : msg.effort;
    if (source.size() != kNumJoints) {
      return false;
    }
    std::copy(source.begin(), source.end(), values.begin());
    return true;
  }
}

template <JointInterface I>
void JointBridge<I>::on_command(const typename CommandMsg::SharedPtr msg) {
  const int64_t received = wall_clock_ns();
  stream_stats_.on_sample(received);
  if (!accepting_samples_.load()) {
    return;
  }
  typename Pipeline::Sample sample;
  if (!command_values(*msg, sample.values) || !all_finite(sample.values)) {
    ++rejected_samples_;
    return;
  }
  sample.stamp_ns = playback_stamp(stamp_to_ns(msg->header.stamp), received);
  if (!samples_.push(sample)) {
    ++dropped_samples_;
  }
}

template <JointInterface I>
bool JointBridge<I>::push_request(const JointMotionRequest &request) {
  std::lock_guard<std::mutex> lock(requests_mutex_);
  return requests_.push(request);
}

template <JointInterface I>
typename JointBridge<I>::Snapshot JointBridge<I>::snapshot() {
  Snapshot snap;
  snapshot_box_.get(snap);
  return snap;
}

template <JointInterface I>
bool JointBridge<I>::start_preset(const std::string &preset, double duration, double amplitude,
                                  std::string &message) {
  if (!accepts_motions()) {
    message = "presets need an active bridge in internal mode";
    return false;
  }
  const Snapshot snap = snapshot();
  if (!snap.valid) {
    message = "control loop not ready";
    return false;
  }
  JointMotionRequest request;
  request.kind = JointMotionRequest::Kind::kStart;
  if (preset == "quintic_joint1") {
    // The demo of the original bridge: joint 1 moves by 0.2 rad in 10 s
    request.motion.type = JointMotion::Type::kQuintic;
    request.motion.goal = snap.position;
    request.motion.goal[0] += amplitude != 0.0 ? amplitude : 0.2;
    request.motion.duration = duration > 0.0 ? duration : 10.0;
  } else if (preset == "sine_joint1") {
    // Joint 1 goes out and back along a cosine, as the sinusoidal demo
    request.motion.type = JointMotion::Type::kCosineBump;
    request.motion.joint = 0;
    request.motion.amplitude = amplitude != 0.0 ? amplitude : 2 * M_PI / 30;
    request.motion.duration = duration > 0.0 ? duration : 10.0;
  } else if (I == JointInterface::kTorques && preset == "gravity_compensation") {
    // The robot floats and can be moved by hand, until /panda/stop_motion or another motion
    request.motion.type = JointMotion::Type::kFree;
  } else {
    message = "unknown preset '" + preset + "' (available: quintic_joint1, sine_joint1" +
              (I == JointInterface::kTorques ? ", gravity_compensation)" : ")");
    return false;
  }
  request.id = next_motion_id_++;
  request.name.set(preset.c_str());
  if (!push_request(request)) {
    message = "request queue full";
    return false;
  }
  message = "preset " + preset + " started as motion " + std::to_string(request.id);
  return true;
}

template <JointInterface I>
rclcpp_action::GoalResponse JointBridge<I>::on_goal(const rclcpp_action::GoalUUID &,
                                                    std::shared_ptr<const JointTraj::Goal> goal) {
  if (!accepts_motions()) {
    RCLCPP_WARN(get_logger(), "Goal rejected: it needs an active bridge in internal mode");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto &q = goal->desired_joint_cmd.position;
  if (q.size() != kNumJoints) {
    RCLCPP_WARN(get_logger(), "Goal rejected: %zu joint positions instead of 7", q.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  const JointLimits limits = fer_joint_limits();
  const double margin = get_parameter("joint_limit_margin").as_double();
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(q[i]) || q[i] < limits.q_min[i] + margin || q[i] > limits.q_max[i] - margin) {
      RCLCPP_WARN(get_logger(), "Goal rejected: joint %zu position %f outside the limits", i + 1, q[i]);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (!(goal->total_time > 0.0) || !std::isfinite(goal->total_time)) {
    RCLCPP_WARN(get_logger(), "Goal rejected: total_time must be positive");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <JointInterface I>
rclcpp_action::CancelResponse JointBridge<I>::on_cancel(const std::shared_ptr<GoalHandle> handle) {
  for (const uint64_t id : goals_.ids_of(handle)) {
    JointMotionRequest cancel;
    cancel.kind = JointMotionRequest::Kind::kCancel;
    cancel.id = id;
    push_request(cancel);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <JointInterface I>
void JointBridge<I>::on_accepted(const std::shared_ptr<GoalHandle> handle) {
  const auto goal = handle->get_goal();
  JointMotionRequest request;
  request.kind = JointMotionRequest::Kind::kStart;
  request.id = next_motion_id_++;
  request.motion.type = JointMotion::Type::kQuintic;
  std::copy(goal->desired_joint_cmd.position.begin(), goal->desired_joint_cmd.position.end(),
            request.motion.goal.begin());
  request.motion.duration = goal->total_time;
  request.name.set("joint_traj");
  if (!push_request(request)) {
    auto result = std::make_shared<typename JointTraj::Result>();
    result->completed = false;
    handle->abort(result);
    return;
  }
  goals_.add(handle, request.id);
}

template <JointInterface I>
void JointBridge<I>::update_goals() {
  if (goals_.empty()) {
    return;
  }
  const Snapshot snap = snapshot();
  goals_.update(snap.motion, snap.valid, control_running() && !control_failed(), get_logger());
}

template <JointInterface I>
void JointBridge<I>::fill_status(BridgeStatus &status) {
  fill_pipeline_status(status, snapshot(), control_running() && !control_failed(), internal_mode_);
  status.rejected_commands += rejected_samples_.load();
  status.dropped_commands = dropped_samples_.load();
  stream_stats_.fill(status);
}

template <JointInterface I>
void JointBridge<I>::process_rt_events() {
  RtEvent event;
  while (events_.pop(event)) {
    log_bridge_event(get_logger(), event, Traits::kQuantity, "rad", last_rejection_log_ns_);
  }
}

template class JointBridge<JointInterface::kPositions>;
template class JointBridge<JointInterface::kVelocities>;
template class JointBridge<JointInterface::kTorques>;

}  // namespace panda_bridge
