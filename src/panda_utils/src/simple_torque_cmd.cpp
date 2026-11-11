// Reference client of the torque bridge in external mode: zero torques at 1 kHz for the given
// duration. The robot compensates gravity, so it floats and can be moved by hand; when the stream
// stops, the bridge damps the motion and holds the position.
//
//   ros2 run panda_utils simple_torque_cmd [duration_s]

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <realtime_tools/realtime_helpers.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  double duration_s = 10.0;
  if (argc == 2) {
    try {
      duration_s = std::stod(argv[1]);
    } catch (const std::exception &e) {
      std::cerr << "Usage: " << argv[0] << " [duration_s]" << std::endl;
      return 1;
    }
  } else if (argc > 2) {
    std::cerr << "Usage: " << argv[0] << " [duration_s]" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("torque_cmd_node");
  rclcpp::QoS torque_cmd_qos(rclcpp::KeepLast(1));
  torque_cmd_qos.reliable();
  torque_cmd_qos.durability_volatile();
  auto cmd_torque_pub = node->create_publisher<sensor_msgs::msg::JointState>("/panda/joint_torque_command", torque_cmd_qos);

  if (realtime_tools::has_realtime_kernel() && !realtime_tools::configure_sched_fifo(99)) {
    RCLCPP_WARN(node->get_logger(), "Could not set SCHED_FIFO: running with the default scheduler");
  }

  sensor_msgs::msg::JointState cmd;
  cmd.name = {"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"};
  // Torques without gravity: zero leaves the robot gravity compensated
  cmd.effort.assign(7, 0.0);

  RCLCPP_INFO(node->get_logger(), "Zero torques for %.1f s: the robot floats and can be moved by hand", duration_s);
  const rclcpp::Time t0 = node->now();
  auto next_wake = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    const rclcpp::Time sample_time = node->now();
    if ((sample_time - t0).seconds() >= duration_s) {
      break;
    }
    cmd.header.stamp = sample_time;
    cmd_torque_pub->publish(cmd);
    next_wake += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(next_wake);
  }
  RCLCPP_INFO(node->get_logger(), "Stream stopped: the bridge damps the motion and holds the position");

  rclcpp::shutdown();
  return 0;
}
