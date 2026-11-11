
#include "sensor_msgs/msg/joint_state.hpp"
#include "panda_utils/constants.hpp"
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <thread>
#include <chrono>
#include <array>
#include <iostream>

double quintic_velocity(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return 0;
  if (t >= t_f)
    return 0;

  double tau = t / t_f;
  double q_cap =
      30 * std::pow(tau, 4) - 60 * std::pow(tau, 3) + 30 * std::pow(tau, 2);
  return (q_f - q_i) * q_cap / t_f;
}




int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Default joint positions
  std::array<double, 7> default_positions = {
    2.2071, 1.2788, 2.1047, -1.9131, -0.1979, 2.5894, 2.3336
  };

  // Parse command-line arguments for desired joint positions
  std::array<double, 7> desired_positions = default_positions;
  if (argc == 8) {  // program name + 7 joint values
    try {
      for (int i = 0; i < 7; i++) {
        desired_positions[i] = std::stod(argv[i + 1]);
      }
    } catch (const std::exception& e) {
      std::cerr << "Error parsing joint positions: " << e.what() << std::endl;
      std::cerr << "Usage: " << argv[0] << " j0 j1 j2 j3 j4 j5 j6" << std::endl;
      return 1;
    }
  } else if (argc > 1) {
    std::cerr << "Expected 7 joint positions, got " << (argc - 1) << std::endl;
    std::cerr << "Usage: " << argv[0] << " j0 j1 j2 j3 j4 j5 j6" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("joint_traj_node");
  rclcpp::Duration traj_duration(5, 0);
  sensor_msgs::msg::JointState::SharedPtr joint_state;

    auto save_joints_state = [&joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state = msg;
    };
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", panda_interface_names::DEFAULT_TOPIC_QOS(), save_joints_state);


    rclcpp::QoS position_cmd_qos(rclcpp::KeepLast(1));
        position_cmd_qos.reliable();
        position_cmd_qos.durability_volatile();
    //create a standard publisher for command positions, to be used in the control loop
    auto cmd_position_pub = node->create_publisher<sensor_msgs::msg::JointState>(
            "/panda/joint_velocity_command",
            position_cmd_qos);

   if (realtime_tools::has_realtime_kernel()){
      if (!realtime_tools::configure_sched_fifo(99)) {
        RCLCPP_WARN(node->get_logger(),
                    "Execute thread: Could not set SCHED_FIFO."
                    " Running with default scheduler.");
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "Execute thread: Set SCHED_FIFO priority.");
      }
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "Execute thread: No real-time kernel detected.");
    }
// Get joint configuration now
    RCLCPP_INFO(node->get_logger(), "Waiting for current joints config");
    sensor_msgs::msg::JointState q0;
    joint_state = sensor_msgs::msg::JointState::SharedPtr();
    while (rclcpp::ok() && !joint_state) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      RCLCPP_INFO(node->get_logger(), "Still waiting current joint state");
    }
    // joint_state = std::make_shared<sensor_msgs::msg::JointState>();

    // joint_state->position.resize(7);
    // joint_state->velocity.resize(7);
    // joint_state->effort.resize(7);
    // for (size_t i = 0; i < 7; i++) {
    //   joint_state->position[i] = 0.0;
    //   joint_state->velocity[i] = 0.0;
    //   joint_state->effort[i] = 0.0;
    // }


    q0 = *joint_state;
    RCLCPP_INFO_STREAM(node->get_logger(), 
                       "Current joint config (q0): ["
                           << q0.position[0] << ", " << q0.position[1] << ", "
                           << q0.position[2] << ", " << q0.position[3] << ", "
                           << q0.position[4] << ", " << q0.position[5] << ", "
                           << q0.position[6] << "]");

    rclcpp::Time t0 = node->now();
    rclcpp::Duration t = rclcpp::Duration(0, 0);
    sensor_msgs::msg::JointState cmd;
    cmd.name = q0.name;
    cmd.velocity.resize(7);
    auto desired_goal = q0;
    for (size_t i = 0; i < 7; i++) {
      desired_goal.position[i] = desired_positions[i];
    }
    // desired_goal.position[0] = 2.2071;
    // desired_goal.position[1] = 1.2788;
    // desired_goal.position[2] = 2.1047;
    // desired_goal.position[3] = -1.9131;
    // desired_goal.position[4] = -0.1979;
    // desired_goal.position[5] = 2.5894;
    // desired_goal.position[6] = 2.3336;
    // desired_goal.position[0] = -2.2071;
    // desired_goal.position[1] = 0.3287;
    // desired_goal.position[2] = 2.53628;
    // desired_goal.position[3] = -1.9728;
    // desired_goal.position[4] = -0.2223;
    // desired_goal.position[5] = 1.687;
    // desired_goal.position[6] = 1.134;
    
    auto next_wake = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
    
    while (rclcpp::ok() && t < traj_duration) {
    // auto old_t = cmd.header.stamp;
    // The stamp is the instant the velocities refer to: the bridge plays the samples back by it
    const rclcpp::Time sample_time = node->now();
    t = sample_time - t0;
    cmd.header.stamp = sample_time;
    for (size_t i = 0; i < 7; i++) {
    cmd.velocity[i] =quintic_velocity(q0.position[i], desired_goal.position[i], t.seconds(), traj_duration.seconds());     
    }
      // // Strong real-time: pubblica senza bloccare
      // if (cmd_position_pub->trylock()) {
      //   cmd_position_pub->msg_ = cmd;
      //   cmd_position_pub->unlockAndPublish();
      // }
      cmd_position_pub->publish(cmd);

      // // loop_rate.sleep();
      std::this_thread::sleep_until(next_wake);
      next_wake += std::chrono::milliseconds(1);
    }

  rclcpp::shutdown();
  return 0;
}