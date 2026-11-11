#include "panda_utils/bridge/bridge_base.hpp"

#include <pthread.h>
#include <sched.h>

#include <algorithm>

#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <realtime_tools/realtime_helpers.hpp>

#include "panda_utils/bridge/state_conversions.hpp"
#include "panda_utils/bridge/types.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/utils_func.hpp"

namespace panda_bridge {

namespace {

constexpr auto kActivationTimeout = std::chrono::milliseconds(2000);
constexpr auto kStopTimeout = std::chrono::milliseconds(3000);
constexpr auto kRecoveryTimeout = std::chrono::milliseconds(15000);

template <std::size_t N>
bool to_array(const std::vector<double> &values, std::array<double, N> &out) {
  if (values.size() != N) {
    return false;
  }
  std::copy(values.begin(), values.end(), out.begin());
  return true;
}

// Column-major homogeneous transform from a translation and a quaternion (w, x, y, z)
std::array<double, 16> homogeneous(const std::array<double, 3> &translation, const std::array<double, 4> &rotation) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) =
      Eigen::Quaterniond(rotation[0], rotation[1], rotation[2], rotation[3]).normalized().toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  std::array<double, 16> out{};
  Eigen::Map<Eigen::Matrix4d>(out.data()) = transform;
  return out;
}

// Configuration in which the fake robot starts
constexpr std::array<double, 7> kFakeRobotPose{0.0, -M_PI_4, 0.0, -3 * M_PI_4, 0.0, M_PI_2, M_PI_4};

}  // namespace

BridgeBase::BridgeBase(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode(node_name, options) {
  command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  services_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recovery_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  declare_common_parameters();
}

BridgeBase::~BridgeBase() {
  // Derived bridges call teardown() in their destructor; this only stops threads left behind
  stop_threads();
}

void BridgeBase::declare_common_parameters() {
  declare_parameter<std::string>("robot_ip", "10.224.20.198");
  // Test only: no connection, a simulated robot follows the commands exactly
  declare_parameter<bool>("fake_robot", false);
  declare_parameter<double>("fake_packet_loss", 0.0);
  declare_parameter<double>("fake_tracking_offset", 0.0);
  declare_parameter<double>("publish_freq", 100.0);
  declare_parameter<bool>("show_details", false);
  declare_parameter<std::vector<double>>("flange_T_ee_translation", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("flange_T_ee_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("F_x_Cload", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<double>("load", 0.0);
  declare_parameter<std::vector<double>>("load_inertia", std::vector<double>(9, 0.0));
  // Collision thresholds and impedances: defaults of the libfranka examples
  declare_parameter<std::vector<double>>("collision_lower_torque_thresholds",
                                         std::vector<double>{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  declare_parameter<std::vector<double>>("collision_upper_torque_thresholds",
                                         std::vector<double>{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  declare_parameter<std::vector<double>>("collision_lower_force_thresholds",
                                         std::vector<double>{20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  declare_parameter<std::vector<double>>("collision_upper_force_thresholds",
                                         std::vector<double>{20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  declare_parameter<std::vector<double>>("joint_impedance",
                                         std::vector<double>{3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0});
  declare_parameter<std::vector<double>>("cartesian_impedance",
                                         std::vector<double>{3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0});
  declare_parameter<double>("critical_height", 0.1);
  declare_parameter<std::vector<std::string>>("joint_names", panda_interface_names::panda_joint_names);
  declare_parameter<std::string>("base_frame", "fer_link0");
  declare_parameter<int>("rt_cpu", -1);
  declare_parameter<bool>("lock_memory", false);
  declare_parameter<int>("executor_rt_priority", 0);
  declare_parameter<bool>("limit_rate", true);
  declare_parameter<double>("lpf_cutoff_frequency", franka::kDefaultCutoffFrequency);
}

bool BridgeBase::read_common_parameters(std::string &error) {
  Params p;
  p.robot_ip = get_parameter("robot_ip").as_string();
  p.fake_robot = get_parameter("fake_robot").as_bool();
  p.publish_freq = get_parameter("publish_freq").as_double();
  p.show_details = get_parameter("show_details").as_bool();
  p.critical_height = get_parameter("critical_height").as_double();
  p.joint_names = get_parameter("joint_names").as_string_array();
  p.base_frame = get_parameter("base_frame").as_string();
  p.rt_cpu = static_cast<int>(get_parameter("rt_cpu").as_int());
  p.lock_memory = get_parameter("lock_memory").as_bool();
  p.limit_rate = get_parameter("limit_rate").as_bool();
  p.lpf_cutoff_frequency = get_parameter("lpf_cutoff_frequency").as_double();

  if (!(p.publish_freq > 0.0 && p.publish_freq <= 1000.0)) {
    error = "publish_freq must be in (0, 1000] Hz";
    return false;
  }
  if (!(p.lpf_cutoff_frequency > 0.0 && p.lpf_cutoff_frequency <= franka::kMaxCutoffFrequency)) {
    error = "lpf_cutoff_frequency must be in (0, 1000] Hz";
    return false;
  }
  if (p.joint_names.size() != kNumJoints) {
    error = "joint_names needs 7 names";
    return false;
  }

  RobotSettings &r = p.robot;
  r.robot_ip = p.robot_ip;
  std::array<double, 3> translation{};
  std::array<double, 4> rotation{};
  const bool arrays_ok =
      to_array(get_parameter("collision_lower_torque_thresholds").as_double_array(), r.lower_torque_thresholds) &&
      to_array(get_parameter("collision_upper_torque_thresholds").as_double_array(), r.upper_torque_thresholds) &&
      to_array(get_parameter("collision_lower_force_thresholds").as_double_array(), r.lower_force_thresholds) &&
      to_array(get_parameter("collision_upper_force_thresholds").as_double_array(), r.upper_force_thresholds) &&
      to_array(get_parameter("joint_impedance").as_double_array(), r.joint_impedance) &&
      to_array(get_parameter("cartesian_impedance").as_double_array(), r.cartesian_impedance) &&
      to_array(get_parameter("F_x_Cload").as_double_array(), r.F_x_Cload) &&
      to_array(get_parameter("load_inertia").as_double_array(), r.load_inertia) &&
      to_array(get_parameter("flange_T_ee_translation").as_double_array(), translation) &&
      to_array(get_parameter("flange_T_ee_rotation").as_double_array(), rotation);
  if (!arrays_ok) {
    error = "wrong size of an array parameter (torque thresholds and joint impedance 7, force thresholds and "
            "cartesian impedance 6, F_x_Cload 3, load_inertia 9, flange_T_ee_translation 3, flange_T_ee_rotation 4)";
    return false;
  }
  r.load_mass = get_parameter("load").as_double();
  r.NE_T_EE = homogeneous(translation, rotation);
  params_ = p;
  return true;
}

BridgeBase::CallbackReturn BridgeBase::on_configure(const rclcpp_lifecycle::State &) {
  std::string error;
  if (!read_common_parameters(error)) {
    RCLCPP_ERROR(get_logger(), "Invalid parameters: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  if (!params_.fake_robot && !realtime_tools::has_realtime_kernel()) {
    RCLCPP_ERROR(get_logger(), "No real-time kernel: libfranka needs one to control the robot");
    return CallbackReturn::FAILURE;
  }
  if (params_.lock_memory) {
    const auto [locked, message] = realtime_tools::lock_memory();
    if (!locked) {
      RCLCPP_WARN(get_logger(), "Could not lock the memory: %s", message.c_str());
    }
  }

  try {
    if (params_.fake_robot) {
      RCLCPP_WARN(get_logger(), "fake_robot: simulated robot, no connection (test only)");
      backend_ = std::make_unique<FakeRobotBackend>(kFakeRobotPose, get_parameter("fake_packet_loss").as_double(),
                                                    get_parameter("fake_tracking_offset").as_double());
    } else {
      RCLCPP_INFO(get_logger(), "Connecting to the robot at %s...", params_.robot_ip.c_str());
      backend_ = std::make_unique<FrankaRobotBackend>(params_.robot);
    }
    const franka::RobotState state = backend_->read_once();
    if (!params_.fake_robot) {
      RCLCPP_INFO(get_logger(), "Connected to the robot at %s", params_.robot_ip.c_str());
    }
    print_initial_franka_state(state, backend_->model(), get_logger(), params_.show_details);
    state_box_.set(state);
  } catch (const franka::Exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to the robot at %s: %s", params_.robot_ip.c_str(), e.what());
    backend_.reset();
    return CallbackReturn::FAILURE;
  }

  auto parameters = get_node_parameters_interface();
  auto topics = get_node_topics_interface();
  const rclcpp::QoS qos = panda_interface_names::DEFAULT_TOPIC_QOS();
  joint_state_pub_ = rclcpp::create_publisher<sensor_msgs::msg::JointState>(
      parameters, topics, panda_interface_names::joint_state_topic_name, qos);
  robot_state_pub_ =
      rclcpp::create_publisher<uclv_panda_interfaces::msg::PandaRobotState>(parameters, topics, "/panda/robot_state", qos);
  pose_read_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PoseStamped>(
      parameters, topics, panda_interface_names::panda_pose_state_topic_name + "_read_ee", qos);
  pose_computed_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PoseStamped>(
      parameters, topics, panda_interface_names::panda_pose_state_topic_name + "_computed_ee", qos);
  status_pub_ = rclcpp::create_publisher<BridgeStatus>(parameters, topics, "/panda/bridge_status", qos);
  recovery_srv_ = create_service<std_srvs::srv::Trigger>(
      "/panda/error_recovery",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) { on_error_recovery(request, response); },
      rclcpp::ServicesQoS(), recovery_group_);

  if (!configure_bridge()) {
    teardown();
    return CallbackReturn::FAILURE;
  }

  robot_thread_exit_ = false;
  publisher_exit_ = false;
  control_wanted_ = false;
  control_failed_ = false;
  robot_thread_ = std::thread(&BridgeBase::robot_thread_main, this);
  publisher_thread_ = std::thread(&BridgeBase::publisher_thread_main, this);
  RCLCPP_INFO(get_logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

BridgeBase::CallbackReturn BridgeBase::on_activate(const rclcpp_lifecycle::State &) {
  if (!backend_) {
    return CallbackReturn::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_.clear();
  }
  stopping_ = false;
  first_cycle_seen_ = false;
  control_failed_ = false;
  control_wanted_ = true;
  const bool settled = wait_for([this] { return first_cycle_seen_.load() || control_failed_.load(); },
                                kActivationTimeout);
  if (!settled || control_failed_) {
    if (!settled) {
      RCLCPP_ERROR(get_logger(), "The control loop did not start within %ld ms",
                   static_cast<long>(kActivationTimeout.count()));
    } else {
      RCLCPP_ERROR(get_logger(), "Activation failed: %s", last_error().c_str());
    }
    stop_control();
    control_failed_ = false;
    return CallbackReturn::FAILURE;
  }
  on_control_started();
  RCLCPP_INFO(get_logger(), "Active: control loop running");
  return CallbackReturn::SUCCESS;
}

BridgeBase::CallbackReturn BridgeBase::on_deactivate(const rclcpp_lifecycle::State &) {
  stop_control();
  control_failed_ = false;
  RCLCPP_INFO(get_logger(), "Inactive: control loop stopped");
  return CallbackReturn::SUCCESS;
}

BridgeBase::CallbackReturn BridgeBase::on_cleanup(const rclcpp_lifecycle::State &) {
  teardown();
  RCLCPP_INFO(get_logger(), "Cleaned up: disconnected from the robot");
  return CallbackReturn::SUCCESS;
}

BridgeBase::CallbackReturn BridgeBase::on_shutdown(const rclcpp_lifecycle::State &) {
  teardown();
  return CallbackReturn::SUCCESS;
}

BridgeBase::CallbackReturn BridgeBase::on_error(const rclcpp_lifecycle::State &) {
  teardown();
  return CallbackReturn::SUCCESS;
}

void BridgeBase::teardown() {
  if (backend_) {
    stop_control();
  }
  cleanup_bridge();
  stop_threads();
  recovery_srv_.reset();
  joint_state_pub_.reset();
  robot_state_pub_.reset();
  pose_read_pub_.reset();
  pose_computed_pub_.reset();
  status_pub_.reset();
  backend_.reset();
}

void BridgeBase::stop_control() {
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    control_wanted_ = false;
    stopping_ = true;
    if (control_running_) {
      request_stop();
    }
  }
  if (!wait_for([this] { return !control_running_.load(); }, kStopTimeout)) {
    RCLCPP_WARN(get_logger(), "Smooth stop not completed within %ld ms: aborting the motion",
                static_cast<long>(kStopTimeout.count()));
    try {
      backend_->stop();
    } catch (const franka::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Stop failed: %s", e.what());
    }
    wait_for([this] { return !control_running_.load(); }, kStopTimeout);
  }
  stopping_ = false;
}

void BridgeBase::stop_threads() {
  publisher_exit_ = true;
  robot_thread_exit_ = true;
  if (robot_thread_.joinable()) {
    robot_thread_.join();
  }
  if (publisher_thread_.joinable()) {
    publisher_thread_.join();
  }
}

void BridgeBase::robot_thread_main() {
  if (params_.rt_cpu >= 0) {
    const auto [pinned, message] = realtime_tools::set_current_thread_affinity(params_.rt_cpu);
    if (pinned) {
      RCLCPP_INFO(get_logger(), "Robot thread pinned to CPU %d", params_.rt_cpu);
    } else {
      RCLCPP_WARN(get_logger(), "Could not pin the robot thread to CPU %d: %s", params_.rt_cpu, message.c_str());
    }
  }
  while (!robot_thread_exit_) {
    if (recovery_requested_.exchange(false)) {
      bool ok = true;
      std::string message = "error recovery done";
      try {
        backend_->automatic_error_recovery();
      } catch (const franka::Exception &e) {
        ok = false;
        message = std::string("error recovery failed: ") + e.what();
      }
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        recovery_ok_ = ok;
        recovery_message_ = message;
        if (ok) {
          last_error_.clear();
        }
      }
      // An active bridge restarts its control loop
      if (ok) {
        control_failed_ = false;
      }
      recovery_done_ = true;
      continue;
    }
    if (control_wanted_ && !control_failed_) {
      run_control_session();
      continue;
    }
    read_idle_state();
  }
}

void BridgeBase::run_control_session() {
  {
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!control_wanted_ || control_failed_) {
      return;
    }
    std::string error;
    if (!prepare_control(error)) {
      set_error(error);
      return;
    }
    rt_cycles_ = 0;
    rt_missed_cycles_ = 0;
    window_sum_ns_ = window_max_ns_ = 0;
    window_count_ = 0;
    control_running_ = true;
  }
  std::string error;
  try {
    run_control(*backend_);
  } catch (const franka::Exception &e) {
    error = e.what();
  } catch (const std::exception &e) {
    error = e.what();
  }
  if (error.empty()) {
    error = finish_reason();
  }
  control_running_ = false;
  if (stopping_) {
    if (!error.empty()) {
      RCLCPP_WARN(get_logger(), "Control loop ended while stopping: %s", error.c_str());
    }
    return;
  }
  set_error(error.empty() ? "control loop finished unexpectedly" : error);
}

void BridgeBase::read_idle_state() {
  try {
    const franka::RobotState state = backend_->read_once();
    state_box_.set(state);
  } catch (const franka::Exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot read the robot state: %s", e.what());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(std::max(1e-3, 1.0 / params_.publish_freq)));
}

void BridgeBase::set_error(const std::string &error) {
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
  }
  control_failed_ = true;
  RCLCPP_ERROR(get_logger(), "Control loop stopped: %s", error.c_str());
}

std::string BridgeBase::last_error() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void BridgeBase::on_error_recovery(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!backend_) {
    response->success = false;
    response->message = "bridge not configured";
    return;
  }
  if (control_running_) {
    response->success = false;
    response->message = "control loop running: nothing to recover";
    return;
  }
  recovery_done_ = false;
  recovery_requested_ = true;
  if (!wait_for([this] { return recovery_done_.load(); }, kRecoveryTimeout)) {
    response->success = false;
    response->message = "error recovery timed out";
    return;
  }
  std::lock_guard<std::mutex> lock(error_mutex_);
  response->success = recovery_ok_;
  response->message = recovery_message_;
  if (recovery_ok_ && control_wanted_) {
    response->message += ", control loop restarting";
  }
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void BridgeBase::publisher_thread_main() {
  // Normal scheduling, also when created from a real-time thread
  sched_param param{};
  param.sched_priority = 0;
  pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);

  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / params_.publish_freq));
  auto next = std::chrono::steady_clock::now();
  while (!publisher_exit_) {
    publish_once();
    next += period;
    std::this_thread::sleep_until(next);
  }
}

void BridgeBase::publish_once() {
  const rclcpp::Time now = this->now();
  franka::RobotState state;
  state_box_.get(state);
  if (state.time.toMSec() != last_published_time_ms_) {
    last_published_time_ms_ = state.time.toMSec();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now;
    joint_state.name = params_.joint_names;
    fill_joint_state(state, joint_state);
    joint_state_pub_->publish(joint_state);

    uclv_panda_interfaces::msg::PandaRobotState robot_state;
    robot_state.header.stamp = now;
    robot_state.header.frame_id = params_.base_frame;
    fill_robot_state(state, robot_state);
    robot_state_pub_->publish(robot_state);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = params_.base_frame;
    pose.pose = geom_utils::get_pose(state.O_T_EE);
    pose_read_pub_->publish(pose);
    if (const franka::Model *model = backend_ ? backend_->model() : nullptr) {
      pose.pose = geom_utils::get_pose(model->pose(franka::Frame::kFlange, state));
      pose_computed_pub_->publish(pose);
    }
  }

  BridgeStatus status;
  status.header.stamp = now;
  status.bridge = get_name();
  status.state = control_failed_ ? BridgeStatus::STATE_ERROR : BridgeStatus::STATE_IDLE;
  LoopStats stats;
  stats_box_.get(stats);
  status.cycles = stats.cycles;
  status.missed_cycles = stats.missed_cycles;
  status.cycle_time_mean_us = stats.mean_us;
  status.cycle_time_max_us = stats.max_us;
  status.control_command_success_rate = state.control_command_success_rate;
  status.last_error = last_error();
  fill_status(status);
  status_pub_->publish(status);

  process_rt_events();
}

void BridgeBase::rt_cycle_begin(const franka::RobotState &state, franka::Duration period) {
  cycle_start_ns_ = monotonic_ns();
  first_cycle_seen_.store(true, std::memory_order_relaxed);
  const uint64_t elapsed_ms = period.toMSec();
  if (elapsed_ms > 0) {
    ++rt_cycles_;
    rt_missed_cycles_ += elapsed_ms - 1;
  }
  (void)state_box_.try_set(state);
}

void BridgeBase::rt_cycle_end() {
  const int64_t duration = monotonic_ns() - cycle_start_ns_;
  window_sum_ns_ += duration;
  window_max_ns_ = std::max(window_max_ns_, duration);
  if (++window_count_ >= 1000) {
    LoopStats stats;
    stats.cycles = rt_cycles_;
    stats.missed_cycles = rt_missed_cycles_;
    stats.mean_us = static_cast<double>(window_sum_ns_) / static_cast<double>(window_count_) * 1e-3;
    stats.max_us = static_cast<double>(window_max_ns_) * 1e-3;
    (void)stats_box_.try_set(stats);
    window_sum_ns_ = window_max_ns_ = 0;
    window_count_ = 0;
  }
}

bool BridgeBase::wait_for(const std::function<bool()> &condition, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}

}  // namespace panda_bridge
