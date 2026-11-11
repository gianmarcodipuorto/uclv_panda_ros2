#pragma once

#include <array>
#include <string>
#include <vector>

#include <franka/model.h>
#include <franka/robot_state.h>
#include <rclcpp/logging.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "uclv_panda_interfaces/msg/panda_robot_state.hpp"

namespace panda_bridge {

// Conversions from libfranka types to messages. Not real-time safe (strings, allocations).

std::vector<std::string> error_names(const franka::Errors &errors);
std::string robot_mode_name(franka::RobotMode mode);

// Positions, velocities and measured torques; the caller sets header and names.
void fill_joint_state(const franka::RobotState &state, sensor_msgs::msg::JointState &msg);
void fill_robot_state(const franka::RobotState &state, uclv_panda_interfaces::msg::PandaRobotState &msg);

// Tables of the state at connection: joint positions, flange and end effector poses and, with
// details, Jacobian, mass matrix, Coriolis vector and Jacobian pseudo-inverse. Without a model
// (fake robot) the flange pose and the details are skipped.
void print_initial_franka_state(const franka::RobotState &state, const franka::Model *model,
                                const rclcpp::Logger &logger, bool showing_details);

}  // namespace panda_bridge
