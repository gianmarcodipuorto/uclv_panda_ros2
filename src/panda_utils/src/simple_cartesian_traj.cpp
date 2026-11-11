// Reference client of the Cartesian bridges in external mode: a rest-to-rest quintic of the end
// effector at 1 kHz, from the commanded pose O_T_EE_d of /panda/robot_state (as in the libfranka
// examples) to a target pose: straight line for the position, slerp for the orientation.
// The target is a pose in the base frame (position and quaternion x, y, z, w, as printed by
// `ros2 topic echo /pose_state_read_ee`), or a displacement that keeps the orientation.
// With --velocity it streams the twist of the same motion to the velocity bridge, otherwise the
// poses to the pose bridge.
//
//   ros2 run panda_utils simple_cartesian_traj x y z qx qy qz qw [duration_s] [--velocity]
//   ros2 run panda_utils simple_cartesian_traj dx dy dz [duration_s] [--velocity]

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <realtime_tools/realtime_helpers.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uclv_panda_interfaces/msg/panda_robot_state.hpp"

namespace {

// Quintic s(t) from 0 to 1 in duration T, and its derivative
void quintic(double t, double T, double &s, double &ds) {
  const double tau = std::min(std::max(t / T, 0.0), 1.0);
  s = tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
  ds = 30 * tau * tau * (1 - 2 * tau + tau * tau) / T;
}

int usage(const char *name) {
  std::cerr << "Usage:\n"
            << "  " << name << " x y z qx qy qz qw [duration_s] [--velocity]   target pose in the base frame\n"
            << "  " << name << " dx dy dz [duration_s] [--velocity]            displacement, orientation kept"
            << std::endl;
  return 1;
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  bool velocity = false;
  std::vector<double> numbers;
  try {
    for (std::size_t i = 1; i < args.size(); ++i) {
      if (args[i] == "--velocity") {
        velocity = true;
      } else {
        numbers.push_back(std::stod(args[i]));
      }
    }
  } catch (const std::exception &) {
    return usage(argv[0]);
  }
  // 7 or 8 numbers: target pose; 3 or 4: displacement. The optional last one is the duration.
  const bool target_pose = numbers.size() == 7 || numbers.size() == 8;
  if (!target_pose && numbers.size() != 3 && numbers.size() != 4) {
    return usage(argv[0]);
  }
  const double duration = numbers.size() == 8 || numbers.size() == 4 ? numbers.back() : 5.0;
  if (!(duration > 0.0)) {
    return usage(argv[0]);
  }

  auto node = rclcpp::Node::make_shared("cartesian_traj_node");
  uclv_panda_interfaces::msg::PandaRobotState::SharedPtr robot_state;
  auto state_sub = node->create_subscription<uclv_panda_interfaces::msg::PandaRobotState>(
      "/panda/robot_state", 10,
      [&robot_state](const uclv_panda_interfaces::msg::PandaRobotState::SharedPtr msg) { robot_state = msg; });

  rclcpp::QoS cmd_qos(rclcpp::KeepLast(1));
  cmd_qos.reliable();
  cmd_qos.durability_volatile();
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/panda/cartesian_position_command", cmd_qos);
  auto twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/panda/cartesian_velocity_command", cmd_qos);

  if (realtime_tools::has_realtime_kernel() && !realtime_tools::configure_sched_fifo(99)) {
    RCLCPP_WARN(node->get_logger(), "Could not set SCHED_FIFO: running with the default scheduler");
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for /panda/robot_state");
  while (rclcpp::ok() && !robot_state) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!robot_state) {
    return 0;
  }

  // Start: O_T_EE_d, column major as Eigen
  const Eigen::Matrix4d T = Eigen::Map<const Eigen::Matrix4d>(robot_state->o_t_ee_d.data());
  const Eigen::Vector3d p0 = T.block<3, 1>(0, 3);
  const Eigen::Quaterniond q0(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
  Eigen::Vector3d p1 = p0 + Eigen::Vector3d(numbers[0], numbers[1], numbers[2]);
  Eigen::Quaterniond q1 = q0;
  if (target_pose) {
    p1 = Eigen::Vector3d(numbers[0], numbers[1], numbers[2]);
    q1 = Eigen::Quaterniond(numbers[6], numbers[3], numbers[4], numbers[5]);  // w, x, y, z
    if (q1.norm() < 0.5) {
      std::cerr << "The quaternion qx qy qz qw must not be zero" << std::endl;
      return 1;
    }
    q1.normalize();
  }
  // Rotation from start to target in the base frame, the shortest one (angle in [0, pi])
  const Eigen::AngleAxisd rotation(q1 * q0.inverse());
  const Eigen::Vector3d r = rotation.axis() * rotation.angle();
  const double distance = (p1 - p0).norm();
  // Peaks of the quintic: 1.875 times the mean speed
  RCLCPP_INFO(node->get_logger(),
              "%s from [%.4f, %.4f, %.4f] to [%.4f, %.4f, %.4f], %.3f m and %.3f rad in %.1f s "
              "(peaks %.3f m/s, %.3f rad/s)",
              velocity ? "Twist" : "Poses", p0.x(), p0.y(), p0.z(), p1.x(), p1.y(), p1.z(), distance,
              rotation.angle(), duration, 1.875 * distance / duration, 1.875 * rotation.angle() / duration);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = robot_state->header.frame_id;
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = robot_state->header.frame_id;

  const rclcpp::Time t0 = node->now();
  auto next_wake = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    // The stamp is the instant the sample refers to: the bridge plays the samples back by it
    const rclcpp::Time sample_time = node->now();
    const double t = (sample_time - t0).seconds();
    if (t > duration) {
      break;
    }
    double s, ds;
    quintic(t, duration, s, ds);
    if (velocity) {
      const Eigen::Vector3d v = (p1 - p0) * ds;
      const Eigen::Vector3d w = r * ds;
      twist.header.stamp = sample_time;
      twist.twist.linear.x = v.x();
      twist.twist.linear.y = v.y();
      twist.twist.linear.z = v.z();
      twist.twist.angular.x = w.x();
      twist.twist.angular.y = w.y();
      twist.twist.angular.z = w.z();
      twist_pub->publish(twist);
    } else {
      const Eigen::Vector3d p = p0 + (p1 - p0) * s;
      const Eigen::Quaterniond q = q0.slerp(s, q1);
      pose.header.stamp = sample_time;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      pose_pub->publish(pose);
    }
    next_wake += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(next_wake);
  }
  RCLCPP_INFO(node->get_logger(), "Done");

  rclcpp::shutdown();
  return 0;
}
