#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "uclv_panda_interfaces/action/gripper.hpp"

#include "panda_utils/bridge/gripper_backend.hpp"

namespace panda_bridge {

// The Franka Hand on the action /panda/gripper and the topic /panda/gripper_state. The gripper has
// a connection of its own: its commands run outside the control loop and never delay it, so every
// bridge serves them, in both modes, active or not.
//   state thread : reads the gripper state, publishes it and the feedback of the running goal
//   worker thread: runs the goals one at a time, each blocking until the fingers stop. A new goal
//                  preempts the running one, a cancel stops it.
class GripperServer {
 public:
  using Gripper = uclv_panda_interfaces::action::Gripper;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Gripper>;

  struct Params {
    double speed{0.1};
    double force{20.0};
    // The two finger joints of /panda/gripper_state
    std::vector<std::string> joint_names;
  };

  // Highest grasping force accepted [N]: the continuous force of the Franka Hand
  static constexpr double kMaxForce = 70.0;
  // Below this opening a CLOSE without an expected width holds nothing [m]
  static constexpr double kEmptyWidth = 0.002;

  // Reads the gripper state once (franka::Exception if the gripper does not answer), then creates
  // the ROS interfaces in the given callback group and starts the threads.
  GripperServer(rclcpp_lifecycle::LifecycleNode &node, std::unique_ptr<GripperBackend> backend, const Params &params,
                const rclcpp::CallbackGroup::SharedPtr &group);
  // Stops a running command (a finished grasp keeps holding) and joins the threads.
  ~GripperServer();

  GripperServer(const GripperServer &) = delete;
  GripperServer &operator=(const GripperServer &) = delete;

 private:
  enum class Outcome { kSucceeded, kAborted, kCanceled };

  rclcpp_action::GoalResponse on_goal(const Gripper::Goal &goal);
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> &handle);
  void on_accepted(const std::shared_ptr<GoalHandle> &handle);
  void state_thread_main();
  void worker_thread_main();
  Outcome execute(const std::shared_ptr<GoalHandle> &handle, Gripper::Result &result);
  // Called with goal_mutex_ held
  void stop_running_command();
  GripperReading latest_reading();
  // First reading taken after this call: it reflects a command that just finished
  GripperReading next_reading();

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<GripperBackend> backend_;
  Params params_;

  // The node keeps callback groups by weak reference: this one lives as long as the server
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp_action::Server<Gripper>::SharedPtr action_server_;

  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  GripperReading reading_;
  uint64_t reading_count_{0};
  std::atomic<bool> state_exit_{false};

  // Goal handles change state only with goal_mutex_ held, so that the feedback published by the
  // state thread never races with the end of the goal
  std::mutex goal_mutex_;
  std::condition_variable goal_cv_;
  std::shared_ptr<GoalHandle> pending_;
  std::shared_ptr<GoalHandle> running_;
  bool worker_exit_{false};

  std::thread state_thread_;
  std::thread worker_thread_;
};

}  // namespace panda_bridge
