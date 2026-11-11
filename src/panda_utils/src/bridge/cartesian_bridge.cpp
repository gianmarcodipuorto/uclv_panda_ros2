#include "panda_utils/bridge/cartesian_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace panda_bridge {

namespace {

using Mode = PipelineTypes::Mode;
using FinishReason = PipelineTypes::FinishReason;
using PipelineState = PipelineTypes::State;

// The commanded end effector stays this much above critical_height
constexpr double kHeightMargin = 0.01;

bool state_finite(const CartesianState &state) {
  return all_finite(state.pose.p) && all_finite(state.pose.q) && std::isfinite(state.elbow) &&
         all_finite(state.velocity);
}

// Normalized orientation of a pose message; false for non-finite values or a zero quaternion
bool pose_values(const geometry_msgs::msg::Pose &pose, std::array<double, 7> &values) {
  values = {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
            pose.orientation.y, pose.orientation.z, pose.orientation.w};
  const double norm = std::sqrt(values[3] * values[3] + values[4] * values[4] + values[5] * values[5] +
                                values[6] * values[6]);
  if (!all_finite(values) || norm < 0.5) {
    return false;
  }
  for (std::size_t i = 3; i < 7; ++i) {
    values[i] /= norm;
  }
  return true;
}

}  // namespace

template <CartesianInterface I>
CartesianBridge<I>::CartesianBridge(const rclcpp::NodeOptions &options) : BridgeBase(Traits::kNodeName, options) {
  declare_parameter<std::string>("mode", "external");
  declare_parameter<std::string>("auto_preset", "");
  declare_parameter<double>("playout_delay_ms", 5.0);
  declare_parameter<double>("stall_timeout_ms", 20.0);
  declare_parameter<double>("command_timeout_ms", 0.0);
  declare_parameter<bool>("clamp", true);
  // Cartesian limits of the commands and of the internal motions, as fractions of the FER limits.
  // Low accelerations keep the joints, moved by the inverse kinematics of the robot, far from
  // their own limits.
  declare_parameter<double>("safe_speed_factor", 0.6);
  declare_parameter<double>("safe_acceleration_factor", 0.3);
  declare_parameter<double>("safe_jerk_factor", 0.3);
  // Elbow: position of joint 3, kept within its limits by joint_limit_margin
  declare_parameter<bool>("use_elbow_control", false);
  declare_parameter<double>("joint_limit_margin", 0.01);
  declare_parameter<std::string>("controller_mode", "cartesian_impedance");
  if constexpr (I == CartesianInterface::kPose) {
    declare_parameter<double>("max_jump", 0.05);
    declare_parameter<double>("max_jump_angle", 0.2);
  }
}

template <CartesianInterface I>
CartesianBridge<I>::~CartesianBridge() {
  teardown();
}

template <CartesianInterface I>
bool CartesianBridge<I>::read_bridge_parameters(PipelineTypes::Config &config, std::string &error) {
  const std::string mode = get_parameter("mode").as_string();
  if (mode == "external") {
    config.mode = Mode::kExternal;
  } else if (mode == "internal") {
    config.mode = Mode::kInternal;
  } else {
    error = "mode must be external or internal, not '" + mode + "'";
    return false;
  }

  const std::string controller_mode = get_parameter("controller_mode").as_string();
  if (controller_mode == "cartesian_impedance") {
    controller_mode_ = franka::ControllerMode::kCartesianImpedance;
  } else if (controller_mode == "joint_impedance") {
    controller_mode_ = franka::ControllerMode::kJointImpedance;
  } else {
    error = "controller_mode must be cartesian_impedance or joint_impedance";
    return false;
  }

  const double playout_delay_ms = get_parameter("playout_delay_ms").as_double();
  const double stall_timeout_ms = get_parameter("stall_timeout_ms").as_double();
  const double command_timeout_ms = get_parameter("command_timeout_ms").as_double();
  const double speed = get_parameter("safe_speed_factor").as_double();
  const double acceleration = get_parameter("safe_acceleration_factor").as_double();
  const double jerk = get_parameter("safe_jerk_factor").as_double();
  CartesianSettings &cartesian = config.cartesian;
  config.clamp = get_parameter("clamp").as_bool();
  cartesian.use_elbow = get_parameter("use_elbow_control").as_bool();
  cartesian.elbow_margin = get_parameter("joint_limit_margin").as_double();
  if constexpr (I == CartesianInterface::kPose) {
    cartesian.max_jump = get_parameter("max_jump").as_double();
    cartesian.max_jump_angle = get_parameter("max_jump_angle").as_double();
  }

  auto factor_ok = [](double f) { return f > 0.0 && f <= 1.0; };
  if (!(playout_delay_ms >= 0.0 && playout_delay_ms <= 1000.0) || !(stall_timeout_ms > 0.0) ||
      !(command_timeout_ms >= 0.0) || !(cartesian.max_jump > 0.0) || !(cartesian.max_jump_angle > 0.0) ||
      !(cartesian.elbow_margin >= 0.0) || !factor_ok(speed) || !factor_ok(acceleration) || !factor_ok(jerk)) {
    error = "playout_delay_ms in [0, 1000], stall_timeout_ms > 0, command_timeout_ms >= 0, max_jump > 0, "
            "max_jump_angle > 0, joint_limit_margin >= 0 and safe_*_factor in (0, 1] are required";
    return false;
  }
  config.playout_delay_ns = static_cast<int64_t>(playout_delay_ms * 1e6);
  config.stall_timeout_ns = static_cast<int64_t>(stall_timeout_ms * 1e6);
  config.command_timeout_ns = static_cast<int64_t>(command_timeout_ms * 1e6);
  cartesian.limits = fer_cartesian_limits(speed, acceleration, jerk);
  cartesian.min_height = params().critical_height + kHeightMargin;
  auto_preset_ = get_parameter("auto_preset").as_string();
  return true;
}

template <CartesianInterface I>
bool CartesianBridge<I>::configure_bridge() {
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

  action_server_ = rclcpp_action::create_server<CartTraj>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "/panda/cart_traj",
      [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CartTraj::Goal> goal) {
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
        CartesianMotionRequest cancel;
        cancel.kind = CartesianMotionRequest::Kind::kCancel;
        cancel.id = 0;
        response->success = push_request(cancel);
        response->message = response->success ? "stopping" : "request queue full";
      },
      rclcpp::ServicesQoS(), services_group());

  goal_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { update_goals(); }, services_group());
  return true;
}

template <CartesianInterface I>
void CartesianBridge<I>::cleanup_bridge() {
  goal_timer_.reset();
  command_sub_.reset();
  preset_srv_.reset();
  stop_srv_.reset();
  goals_.abort_all();
  action_server_.reset();
}

template <CartesianInterface I>
bool CartesianBridge<I>::prepare_control(std::string &error) {
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
  CartesianMotionRequest request;
  while (requests_.pop(request)) {
  }
  snapshot_box_.set(Snapshot{});
  rt_previous_ = Snapshot{};
  non_finite_command_ = false;
  stop_requested_ = false;
  internal_mode_ = config_.mode == Mode::kInternal;
  RCLCPP_INFO(get_logger(), "Control run in %s mode%s", internal_mode_ ? "internal" : "external",
              config_.cartesian.use_elbow ? ", elbow commanded" : "");
  return true;
}

template <CartesianInterface I>
void CartesianBridge<I>::on_control_started() {
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

template <CartesianInterface I>
void CartesianBridge<I>::run_control(RobotBackend &robot) {
  accepting_samples_ = config_.mode == Mode::kExternal;
  const auto callback = [this](const franka::RobotState &state, franka::Duration period) {
    return rt_update(state, period);
  };
  try {
    if constexpr (I == CartesianInterface::kPose) {
      robot.control_cartesian_pose(callback, controller_mode_, params().limit_rate, params().lpf_cutoff_frequency);
    } else {
      robot.control_cartesian_velocities(callback, controller_mode_, params().limit_rate,
                                         params().lpf_cutoff_frequency);
    }
  } catch (...) {
    accepting_samples_ = false;
    throw;
  }
  accepting_samples_ = false;
}

template <CartesianInterface I>
CartesianFeedback CartesianBridge<I>::rt_feedback(const franka::RobotState &state) const {
  CartesianFeedback robot;
  robot.O_T_EE_c = state.O_T_EE_c;
  robot.O_dP_EE_c = state.O_dP_EE_c;
  robot.O_ddP_EE_c = state.O_ddP_EE_c;
  robot.elbow_c = state.elbow_c;
  robot.delbow_c = state.delbow_c;
  robot.ddelbow_c = state.ddelbow_c;
  return robot;
}

template <CartesianInterface I>
typename CartesianBridge<I>::FrankaCommand CartesianBridge<I>::franka_command(const CartesianState &command) const {
  if constexpr (I == CartesianInterface::kPose) {
    const std::array<double, 16> pose = matrix_from_pose(command.pose);
    return config_.cartesian.use_elbow ? franka::CartesianPose(pose, {command.elbow, elbow_sign_})
                                       : franka::CartesianPose(pose);
  } else {
    const std::array<double, 6> twist{command.velocity[0], command.velocity[1], command.velocity[2],
                                      command.velocity[3], command.velocity[4], command.velocity[5]};
    // The elbow is commanded by position in both interfaces
    return config_.cartesian.use_elbow ? franka::CartesianVelocities(twist, {command.elbow, elbow_sign_})
                                       : franka::CartesianVelocities(twist);
  }
}

template <CartesianInterface I>
typename CartesianBridge<I>::FrankaCommand CartesianBridge<I>::hold_command(const franka::RobotState &state) const {
  if constexpr (I == CartesianInterface::kPose) {
    return config_.cartesian.use_elbow ? franka::CartesianPose(state.O_T_EE_c, state.elbow_c)
                                       : franka::CartesianPose(state.O_T_EE_c);
  } else {
    return config_.cartesian.use_elbow ? franka::CartesianVelocities(std::array<double, 6>{}, state.elbow_c)
                                       : franka::CartesianVelocities(std::array<double, 6>{});
  }
}

template <CartesianInterface I>
typename CartesianBridge<I>::FrankaCommand CartesianBridge<I>::rt_update(const franka::RobotState &state,
                                                                         franka::Duration period) {
  rt_cycle_begin(state, period);
  const int64_t wall = wall_clock_ns();
  const int64_t robot_time = static_cast<int64_t>(state.time.toMSec()) * kCycleNs;
  const uint64_t elapsed = period.toMSec();
  if (elapsed == 0) {
    // First cycle of the control run: start from the last command of the robot
    playback_clock_.reset(wall, robot_time);
    pipeline_.reset(rt_feedback(state), wall);
    elbow_sign_ = state.elbow_c[1];
  } else {
    // Playback on the robot clock, aligned to the clock of the stamps
    const int64_t now = playback_clock_.update(wall, robot_time);
    typename Pipeline::Sample sample;
    while (samples_.pop(sample)) {
      pipeline_.push_sample(sample);
    }
    CartesianMotionRequest request;
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

  bool finished = pipeline_.state() == PipelineState::kFinished;
  FrankaCommand command = hold_command(state);
  if (state_finite(pipeline_.command())) {
    command = franka_command(pipeline_.command());
  } else {
    // Never expected: keep the robot where it is
    (void)events_.push(RtEvent{kNonFiniteCommand, 0, 0.0});
    non_finite_command_ = true;
    finished = true;
  }
  rt_cycle_end();
  return finished ? franka::MotionFinished(command) : command;
}

template <CartesianInterface I>
void CartesianBridge<I>::rt_publish_snapshot() {
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

template <CartesianInterface I>
std::string CartesianBridge<I>::finish_reason() const {
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

template <CartesianInterface I>
bool CartesianBridge<I>::command_values(const CommandMsg &msg, typename Pipeline::Values &values) {
  if constexpr (I == CartesianInterface::kPose) {
    return pose_values(msg.pose, values);
  } else {
    values = {msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z,
              msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z};
    return true;
  }
}

template <CartesianInterface I>
void CartesianBridge<I>::on_command(const typename CommandMsg::SharedPtr msg) {
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

template <CartesianInterface I>
bool CartesianBridge<I>::push_request(const CartesianMotionRequest &request) {
  std::lock_guard<std::mutex> lock(requests_mutex_);
  return requests_.push(request);
}

template <CartesianInterface I>
typename CartesianBridge<I>::Snapshot CartesianBridge<I>::snapshot() {
  Snapshot snap;
  snapshot_box_.get(snap);
  return snap;
}

template <CartesianInterface I>
bool CartesianBridge<I>::start_preset(const std::string &preset, double duration, double amplitude,
                                      std::string &message) {
  if (!accepts_motions()) {
    message = "presets need an active bridge in internal mode";
    return false;
  }
  if (!snapshot().valid) {
    message = "control loop not ready";
    return false;
  }
  CartesianMotionRequest request;
  request.kind = CartesianMotionRequest::Kind::kStart;
  if (preset == "line_x") {
    // Out along the x axis of the base and back
    request.motion.type = CartesianMotion::Type::kLineBump;
    request.motion.axis = 0;
    request.motion.amplitude = amplitude != 0.0 ? amplitude : 0.1;
    request.motion.duration = duration > 0.0 ? duration : 6.0;
  } else if (preset == "circle_xy") {
    // One turn of a circle in the horizontal plane, starting and ending where the robot is
    request.motion.type = CartesianMotion::Type::kCircle;
    request.motion.amplitude = amplitude != 0.0 ? amplitude : 0.05;
    request.motion.duration = duration > 0.0 ? duration : 10.0;
  } else {
    message = "unknown preset '" + preset + "' (available: line_x, circle_xy)";
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

template <CartesianInterface I>
rclcpp_action::GoalResponse CartesianBridge<I>::on_goal(const rclcpp_action::GoalUUID &,
                                                        std::shared_ptr<const CartTraj::Goal> goal) {
  if (!accepts_motions()) {
    RCLCPP_WARN(get_logger(), "Goal rejected: it needs an active bridge in internal mode");
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::array<double, 7> pose{};
  if (!pose_values(goal->desired_pose, pose)) {
    RCLCPP_WARN(get_logger(), "Goal rejected: invalid pose (non-finite values or zero quaternion)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const double min_height = params().critical_height + kHeightMargin;
  if (pose[2] < min_height) {
    RCLCPP_WARN(get_logger(), "Goal rejected: height %.3f m below the minimum of %.3f m", pose[2], min_height);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!(goal->total_time > 0.0) || !std::isfinite(goal->total_time)) {
    RCLCPP_WARN(get_logger(), "Goal rejected: total_time must be positive");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Without elbow control desired_elbow is ignored; with it, NaN keeps the elbow
  if (get_parameter("use_elbow_control").as_bool() && std::isfinite(goal->desired_elbow)) {
    const CartesianLimits limits = fer_cartesian_limits();
    const double margin = get_parameter("joint_limit_margin").as_double();
    if (goal->desired_elbow < limits.elbow_min + margin || goal->desired_elbow > limits.elbow_max - margin) {
      RCLCPP_WARN(get_logger(), "Goal rejected: elbow %f outside the limits of joint 3", goal->desired_elbow);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <CartesianInterface I>
rclcpp_action::CancelResponse CartesianBridge<I>::on_cancel(const std::shared_ptr<GoalHandle> handle) {
  for (const uint64_t id : goals_.ids_of(handle)) {
    CartesianMotionRequest cancel;
    cancel.kind = CartesianMotionRequest::Kind::kCancel;
    cancel.id = id;
    push_request(cancel);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <CartesianInterface I>
void CartesianBridge<I>::on_accepted(const std::shared_ptr<GoalHandle> handle) {
  const auto goal = handle->get_goal();
  std::array<double, 7> pose{};
  pose_values(goal->desired_pose, pose);
  CartesianMotionRequest request;
  request.kind = CartesianMotionRequest::Kind::kStart;
  request.id = next_motion_id_++;
  request.motion.type = CartesianMotion::Type::kQuintic;
  request.motion.goal.p = {pose[0], pose[1], pose[2]};
  request.motion.goal.q = {pose[3], pose[4], pose[5], pose[6]};
  // NaN keeps the elbow
  request.motion.goal_elbow = goal->desired_elbow;
  request.motion.duration = goal->total_time;
  request.name.set("cart_traj");
  if (!push_request(request)) {
    auto result = std::make_shared<typename CartTraj::Result>();
    result->completed = false;
    result->message = "request queue full";
    handle->abort(result);
    return;
  }
  goals_.add(handle, request.id);
}

template <CartesianInterface I>
void CartesianBridge<I>::update_goals() {
  if (goals_.empty()) {
    return;
  }
  const Snapshot snap = snapshot();
  goals_.update(snap.motion, snap.valid, control_running() && !control_failed(), get_logger());
}

template <CartesianInterface I>
void CartesianBridge<I>::fill_status(BridgeStatus &status) {
  fill_pipeline_status(status, snapshot(), control_running() && !control_failed(), internal_mode_);
  status.rejected_commands += rejected_samples_.load();
  status.dropped_commands = dropped_samples_.load();
  stream_stats_.fill(status);
}

template <CartesianInterface I>
void CartesianBridge<I>::process_rt_events() {
  RtEvent event;
  while (events_.pop(event)) {
    log_bridge_event(get_logger(), event, Traits::kQuantity, "m", last_rejection_log_ns_);
  }
}

template class CartesianBridge<CartesianInterface::kPose>;
template class CartesianBridge<CartesianInterface::kVelocities>;

}  // namespace panda_bridge
