
#include "uclv_panda_interfaces/msg/joints_commanded_positions.hpp"
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

double quintic(double q_i, double q_f, double t, double t_f) {
  if (t <= 0)
    return q_i;
  if (t >= t_f)
    return q_f;

  double tau = t / t_f;
  double q_cap =
      6 * std::pow(tau, 5) - 15 * std::pow(tau, 4) + 10 * std::pow(tau, 3);
  return q_i + (q_f - q_i) * q_cap;
}




int main(int argc, char **argv) {
  rclcpp::init(argc, argv);


  auto node = rclcpp::Node::make_shared("joint_traj_node");
  rclcpp::Duration traj_duration(10, 0);
  sensor_msgs::msg::JointState::SharedPtr joint_state;

    auto save_joints_state = [&joint_state](const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state = msg;
    };
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", panda_interface_names::DEFAULT_TOPIC_QOS(), save_joints_state);
  
    //auto cmd_position_pub = std::make_shared<realtime_tools::RealtimePublisher<panda_interfaces::msg::JointsCommandedPositions>>(node->create_publisher<panda_interfaces::msg::JointsCommandedPositions>(
            // "/panda/joint_position_command",
            // panda_interface_names::DEFAULT_TOPIC_QOS()));


    rclcpp::QoS position_cmd_qos(rclcpp::KeepLast(1));
        position_cmd_qos.reliable();
        position_cmd_qos.durability_volatile();
    //create a standard publisher for command positions, to be used in the control loop
    auto cmd_position_pub = node->create_publisher<uclv_panda_interfaces::msg::JointsCommandedPositions>(
            "/panda/joint_position_command",
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
    uclv_panda_interfaces::msg::JointsCommandedPositions cmd;
    auto desired_goal = q0;
    desired_goal.position[0] += 0.2;

    
    auto next_wake = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
    
    while (rclcpp::ok() && t < traj_duration) {
    // auto old_t = cmd.header.stamp;
    t = node->now() - t0;
    cmd.header.stamp = node->now();  // Subito dopo il calcolo di t
    for (size_t i = 0; i < 7; i++) {
    cmd.positions[i] =quintic(q0.position[i], desired_goal.position[i], t.seconds(), traj_duration.seconds());     
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