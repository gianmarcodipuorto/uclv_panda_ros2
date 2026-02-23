#pragma once

#include <rclcpp/qos.hpp>
#include <string>
#include "tf2_eigen/tf2_eigen.hpp"

extern const rclcpp::Duration max_tf_age;
extern const double robot_radius_area; // meters

namespace panda_interface_names {

/////////////////////////////////////////////////////////////
const std::string pd_grav_controller_node_name{"pd_plus_gravity_controller"}; //N.u.
const std::string inverse_dynamics_controller_node_name{"inverse_dynamics_controller"};
const std::string controller_manager_node_name{"controller_manager"}; //N.u.
const std::string clik_node_name{"clik_cmd_pub"}; //N.u.
const std::string cart_traj_node_name{"cart_traj_server"};
const std::string exponential_stop_traj_node_name{"exponential_stop_server"};
const std::string loop_cart_traj_node_name{"loop_cart_traj_server"};


/////////////////////////////////////////////////////////////
// Joint space
const std::string panda_effort_cmd_topic_name{"/panda/cmd/effort"};
const std::string panda_pos_cmd_topic_name{"/panda/cmd/joint_pos"}; //N.u.
const std::string panda_joint_cmd_topic_name{"/panda/cmd/joint_cmd"};
// Cartesian space
const std::string panda_pose_cmd_topic_name{"/panda/cmd/pose"};
const std::string panda_twist_cmd_topic_name{"/panda/cmd/twist"}; 
const std::string panda_accel_cmd_topic_name{"/panda/cmd/accel"};

const std::string joint_state_topic_name{"/joint_states"};
const std::string panda_pose_state_topic_name{"/pose_state"};
const std::string panda_frame_poses_topic_name{"/frame_poses"}; 
const std::string torque_sensor_topic_name{"/tau_sensors"};
const std::string start_and_stop_clik_topic_name{"/clik_ctrl"}; //N.u.
const std::string min_singular_value_topic_name{"/sigma_min"}; 
const std::string pose_error_topic_name{"/pose_error"}; //N.u.

/////////////////////////////////////////////////////////////
const std::string panda_traj_move_action_name{"joint_traj_action"};
const std::string panda_cart_move_action_name{"cart_traj_action"};

const std::vector<std::string> panda_joint_names = {
    "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
    "fer_joint5", "fer_joint6", "fer_joint7",
};

const std::vector<std::string> panda_link_names = {
    "fer_link0", "fer_link1", "fer_link2", "fer_link3", "fer_link4",
    "fer_link5", "fer_link6", "fer_link7", "fer_link8",
};

const inline rclcpp::QoS DEFAULT_TOPIC_QOS() {
  rclcpp::QoS qos(1);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable)
      .history(rclcpp::HistoryPolicy::KeepLast)
      .durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

const inline rclcpp::QoS CONTROLLER_SUBSCRIBER_QOS() {
  rclcpp::QoS qos(1);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable)
      .history(rclcpp::HistoryPolicy::KeepLast)
      .durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

const inline rclcpp::QoS CONTROLLER_PUBLISHER_QOS() {
  rclcpp::QoS qos(1);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort)
      .history(rclcpp::HistoryPolicy::KeepLast)
      .durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

} // namespace panda_interface_names
namespace panda_constants {
const std::string panda_model_effort{"/models/panda/panda_fer.urdf"};

//i limiti sono quelli del panda
const Eigen::Vector<double, 7> effort_limits{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

const Eigen::Vector<double, 7> effort_speed_limits{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};

const Eigen::Vector<double, 7> joint_min_limits = Eigen::Vector<double, 7>{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};

const Eigen::Vector<double, 7> joint_max_limits = Eigen::Vector<double, 7>{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

const Eigen::Vector<double, 7> velocity_limits = Eigen::Vector<double, 7>{2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};

const Eigen::Vector<double, 7> acceleration_limits = Eigen::Vector<double, 7>{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};

const Eigen::Vector<double, 2> cartesian_velocity_limits = Eigen::Vector<double, 2>{1.7,2.5}; //m/s and rad/s

} // namespace panda_constants
