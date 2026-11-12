#pragma once

#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>

namespace panda_bridge {

// main() of the bridge executables.
template <typename Bridge>
int run_bridge(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Nothing outside the bridge ends it abruptly while it controls the robot. Started by the GUI,
  // its log goes to a pipe: if the GUI dies, writing to it must not kill the bridge (SIGPIPE).
  // Closing its terminal (SIGHUP) stops it smoothly, as Ctrl+C (SIGINT) does.
  std::signal(SIGPIPE, SIG_IGN);
  std::signal(SIGHUP, [](int) { std::raise(SIGINT); });
  auto node = std::make_shared<Bridge>();

  // Executor threads inherit the scheduling of this thread; the publisher thread resets itself to
  // normal priority, the control loop gets the highest priority from libfranka
  const auto priority = static_cast<int>(node->get_parameter("executor_rt_priority").as_int());
  if (priority > 0 && !realtime_tools::configure_sched_fifo(priority)) {
    RCLCPP_WARN(node->get_logger(), "Could not set SCHED_FIFO priority %d for the executor", priority);
  }

  // One thread per callback group: lifecycle, command stream, actions and services, recovery,
  // gripper
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

}  // namespace panda_bridge
