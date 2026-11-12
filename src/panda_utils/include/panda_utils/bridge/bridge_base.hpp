#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_panda_interfaces/msg/bridge_status.hpp"
#include "uclv_panda_interfaces/msg/panda_robot_state.hpp"

#include "panda_utils/bridge/gripper_server.hpp"
#include "panda_utils/bridge/robot_backend.hpp"
#include "panda_utils/bridge/rt_exchange.hpp"

namespace panda_bridge {

// Lifecycle node shared by the bridges. It owns the connection to the robot and one robot
// thread, which runs the control loop of the bridge while active and reads the robot state
// otherwise, so that the state is published as soon as the node is configured.
//
//   configure : connect, apply collision/impedance/load/end effector settings, publish the state
//   activate  : start the control loop; succeeds once its first cycle ran
//   deactivate: smooth stop inside the control loop, then back to reading the state
//   cleanup   : stop the threads and disconnect
//
// Errors of the control loop (reflexes, safety stops) leave the node active, with state "error"
// in /panda/bridge_status, until /panda/error_recovery restarts it.
// From configure to cleanup the node also serves the Franka Hand (GripperServer), on a connection
// of its own: in every bridge and mode, whatever the control loop is doing.
// Derived destructors must call teardown() while their members still exist.
class BridgeBase : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using BridgeStatus = uclv_panda_interfaces::msg::BridgeStatus;

  BridgeBase(const std::string &node_name, const rclcpp::NodeOptions &options);
  ~BridgeBase() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  // Callback groups served by separate executor threads (use a MultiThreadedExecutor)
  rclcpp::CallbackGroup::SharedPtr command_group() const { return command_group_; }
  rclcpp::CallbackGroup::SharedPtr services_group() const { return services_group_; }

 protected:
  struct Params {
    std::string robot_ip;
    bool fake_robot{false};
    double publish_freq{100.0};
    bool show_details{false};
    double critical_height{0.1};
    std::vector<std::string> joint_names;
    std::string base_frame;
    int rt_cpu{-1};
    bool lock_memory{false};
    bool limit_rate{true};
    double lpf_cutoff_frequency{100.0};
    RobotSettings robot;
    bool use_gripper{true};
    GripperServer::Params gripper;
  };

  // --- implemented by the bridges ---
  // Read the bridge parameters and create its ROS interfaces. false aborts the configuration.
  virtual bool configure_bridge() = 0;
  virtual void cleanup_bridge() = 0;
  // Before every control run (activation, restart after a recovery), in the robot thread.
  virtual bool prepare_control(std::string &error) = 0;
  // After a successful activation, once the control loop runs.
  virtual void on_control_started() {}
  // Blocking: runs the libfranka control loop until the motion finishes.
  virtual void run_control(RobotBackend &robot) = 0;
  // Smooth stop, then finish the motion. Must be real-time safe for the loop (atomic flag).
  virtual void request_stop() = 0;
  // Why the control loop finished by itself. Called in the robot thread after run_control().
  virtual std::string finish_reason() const = 0;
  // Publisher thread: bridge fields of the status, and logging of the real-time events.
  virtual void fill_status(BridgeStatus &status) = 0;
  virtual void process_rt_events() = 0;

  // --- for the real-time callback of the bridges ---
  // Call at the start and at the end of every control cycle: statistics and state snapshot.
  void rt_cycle_begin(const franka::RobotState &state, franka::Duration period);
  void rt_cycle_end();
  bool below_critical_height(const franka::RobotState &state) const {
    return state.O_T_EE[14] < params_.critical_height;
  }

  // Stops the control loop and the threads, destroys the interfaces and disconnects.
  void teardown();

  const Params &params() const { return params_; }
  bool control_running() const { return control_running_.load(); }
  bool control_failed() const { return control_failed_.load(); }
  int64_t stamp_to_ns(const builtin_interfaces::msg::Time &stamp) const {
    return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
  }

 private:
  struct LoopStats {
    uint64_t cycles{0};
    uint64_t missed_cycles{0};
    double mean_us{0.0};
    double max_us{0.0};
  };

  void declare_common_parameters();
  bool read_common_parameters(std::string &error);
  // Connects to the Franka Hand; without one the bridge works anyway, without /panda/gripper
  void start_gripper();
  void robot_thread_main();
  void run_control_session();
  void read_idle_state();
  void publisher_thread_main();
  void publish_once();
  void set_error(const std::string &error);
  std::string last_error() const;
  void stop_control();
  void stop_threads();
  void on_error_recovery(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  static bool wait_for(const std::function<bool()> &condition, std::chrono::milliseconds timeout);

  Params params_;
  std::unique_ptr<RobotBackend> backend_;

  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr services_group_;
  rclcpp::CallbackGroup::SharedPtr recovery_group_;
  rclcpp::CallbackGroup::SharedPtr gripper_group_;
  std::unique_ptr<GripperServer> gripper_;

  // Plain publishers, not lifecycle-managed: the state is published while inactive too
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<uclv_panda_interfaces::msg::PandaRobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_read_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_computed_pub_;
  rclcpp::Publisher<BridgeStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_srv_;

  std::thread robot_thread_;
  std::thread publisher_thread_;
  std::atomic<bool> robot_thread_exit_{false};
  std::atomic<bool> publisher_exit_{false};

  // Robot thread coordination. session_mutex_ makes "start a control run" and "stop it" atomic
  // with respect to each other, so that a stop request cannot fall before prepare_control()
  std::mutex session_mutex_;
  std::atomic<bool> control_wanted_{false};
  std::atomic<bool> control_running_{false};
  std::atomic<bool> control_failed_{false};
  std::atomic<bool> first_cycle_seen_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> recovery_requested_{false};
  std::atomic<bool> recovery_done_{false};
  bool recovery_ok_{false};
  std::string recovery_message_;
  mutable std::mutex error_mutex_;
  std::string last_error_;

  // Written by the robot thread (control loop or idle reads), read by the publisher thread
  SnapshotBox<franka::RobotState> state_box_;
  SnapshotBox<LoopStats> stats_box_;
  uint64_t last_published_time_ms_{UINT64_MAX};

  // Real-time loop statistics
  int64_t cycle_start_ns_{0};
  uint64_t rt_cycles_{0};
  uint64_t rt_missed_cycles_{0};
  int64_t window_sum_ns_{0};
  int64_t window_max_ns_{0};
  uint64_t window_count_{0};
};

}  // namespace panda_bridge
