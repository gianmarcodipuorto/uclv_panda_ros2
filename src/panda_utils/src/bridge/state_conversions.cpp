#include "panda_utils/bridge/state_conversions.hpp"

#include <iomanip>
#include <sstream>

#include <Eigen/Dense>

#include "panda_utils/utils_func.hpp"

namespace panda_bridge {

std::vector<std::string> error_names(const franka::Errors &errors) {
  // libfranka formats the active errors as ["name_a", "name_b"]
  std::vector<std::string> names;
  const std::string text = static_cast<std::string>(errors);
  std::size_t begin = text.find('"');
  while (begin != std::string::npos) {
    const std::size_t end = text.find('"', begin + 1);
    if (end == std::string::npos) {
      break;
    }
    names.push_back(text.substr(begin + 1, end - begin - 1));
    begin = text.find('"', end + 1);
  }
  return names;
}

std::string robot_mode_name(franka::RobotMode mode) {
  switch (mode) {
    case franka::RobotMode::kIdle:
      return "idle";
    case franka::RobotMode::kMove:
      return "move";
    case franka::RobotMode::kGuiding:
      return "guiding";
    case franka::RobotMode::kReflex:
      return "reflex";
    case franka::RobotMode::kUserStopped:
      return "user_stopped";
    case franka::RobotMode::kAutomaticErrorRecovery:
      return "automatic_error_recovery";
    case franka::RobotMode::kOther:
    default:
      return "other";
  }
}

void fill_joint_state(const franka::RobotState &state, sensor_msgs::msg::JointState &msg) {
  msg.position.assign(state.q.begin(), state.q.end());
  msg.velocity.assign(state.dq.begin(), state.dq.end());
  msg.effort.assign(state.tau_J.begin(), state.tau_J.end());
}

void fill_robot_state(const franka::RobotState &state, uclv_panda_interfaces::msg::PandaRobotState &msg) {
  msg.q = state.q;
  msg.q_d = state.q_d;
  msg.dq = state.dq;
  msg.dq_d = state.dq_d;
  msg.tau_j = state.tau_J;
  msg.tau_j_d = state.tau_J_d;
  msg.tau_ext_hat_filtered = state.tau_ext_hat_filtered;
  msg.o_t_ee = state.O_T_EE;
  msg.o_t_ee_d = state.O_T_EE_d;
  msg.o_f_ext_hat_k = state.O_F_ext_hat_K;
  msg.elbow = state.elbow;
  msg.control_command_success_rate = state.control_command_success_rate;
  msg.robot_mode = robot_mode_name(state.robot_mode);
  msg.current_errors = error_names(state.current_errors);
  msg.last_motion_errors = error_names(state.last_motion_errors);
  msg.robot_time = state.time.toSec();
}

void fill_model(const franka::Model &model, const franka::RobotState &state,
                uclv_panda_interfaces::msg::PandaRobotState &msg) {
  msg.has_model = true;
  msg.jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
  msg.mass = model.mass(state);
  msg.coriolis = model.coriolis(state);
  msg.gravity = model.gravity(state);
}

namespace {

void print_pose_box(const rclcpp::Logger &logger, const char *title, const std::array<double, 16> &transform) {
  const geometry_msgs::msg::Pose current_pose = geom_utils::get_pose(transform);
  std::ostringstream pose_box;
  pose_box << "┌──────────────────────────────────────────────┐\n";
  pose_box << title;
  pose_box << "└──────────────────────────────────────────────┘\n";

  pose_box << "Position (x,y,z): ["
          << std::fixed << std::setprecision(8)
          << current_pose.position.x << ", "
          << current_pose.position.y << ", "
          << current_pose.position.z << " ]\n";

  pose_box << "Orientation (qw,qx,qy,qz): ["
          << std::fixed << std::setprecision(8)
          << current_pose.orientation.w << ", "
          << current_pose.orientation.x << ", "
          << current_pose.orientation.y << ", "
          << current_pose.orientation.z << " ]\n";
  RCLCPP_INFO_STREAM(logger, "\n" << pose_box.str());
}

}  // namespace

void print_initial_franka_state(const franka::RobotState &state, const franka::Model *model,
                                const rclcpp::Logger &logger, bool showing_details) {
  //Print Joint states
  std::ostringstream joints_box;
  joints_box << "┌──────────────────────────────┐\n";
  joints_box << "│        Joint positions       │\n";
  joints_box << "├──────────────────────────────┤\n";
  for (size_t i = 0; i < state.q.size(); i++) {
    joints_box << "│ Joint " << (i + 1) << ": " << state.q[i];
    std::string line = joints_box.str();
    auto last_nl = line.find_last_of('\n');
    auto current_len = (last_nl == std::string::npos) ? line.size() : (line.size() - last_nl - 1);
    if (current_len < 33) {
      joints_box << std::string(33 - current_len, ' ');
    }
    joints_box << "│\n";
  }
  joints_box << "└──────────────────────────────┘";
  RCLCPP_INFO_STREAM(logger, "\n" << joints_box.str());

  if (model != nullptr) {
    print_pose_box(logger, "│               PANDA FLANGE POSE              │\n", model->pose(franka::Frame::kFlange, state));
  }
  print_pose_box(logger, "│           PANDA END EFFECTOR POSE            │\n", state.O_T_EE);

  if (showing_details && model != nullptr) {
    const Eigen::Matrix<double, 6, 7> jacobian =
        geom_utils::get_jacobian(model->zeroJacobian(franka::Frame::kFlange, state));
    RCLCPP_INFO_STREAM(logger, "Current jacobian: \n[" << jacobian << "]");

    // B(q) letta dal modello
    std::array<double, 49> mass_matrix_raw = model->mass(state);
    Eigen::Matrix<double, 7, 7> mass_matrix;
    for (size_t i = 0; i < 7; i++) {
      for (size_t j = 0; j < 7; j++) {
        mass_matrix(j, i) = mass_matrix_raw[i * 7 + j];
      }
    }

    // C(q, qdot) calcolata dal modello
    std::array<double, 7> coriolis_raw = model->coriolis(state);
    Eigen::Vector<double, 7> coriolis;
    for (size_t i = 0; i < 7; i++) {
      coriolis(i) = coriolis_raw[i];
    }

    const Eigen::Matrix<double, 7, 6> jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    RCLCPP_INFO_STREAM(logger, "Current mass matrix: \n[" << mass_matrix << "]");

    RCLCPP_INFO_STREAM(logger, "Current coriolis vector: \n[" << coriolis << "]");

    RCLCPP_INFO_STREAM(logger, "Current pseudoinv: \n[" << jacobian_pinv << "]");
  }
}

}  // namespace panda_bridge
