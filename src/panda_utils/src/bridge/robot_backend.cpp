#include "panda_utils/bridge/robot_backend.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

#include <franka/exception.h>

#include "panda_utils/bridge/cartesian_types.hpp"

namespace panda_bridge {

namespace {

// Inertia seen by each joint of the fake robot, motors included (kg m^2)
constexpr std::array<double, 7> kFakeInertia{1.0, 1.0, 0.5, 0.5, 0.2, 0.2, 0.1};

}  // namespace

FrankaRobotBackend::FrankaRobotBackend(const RobotSettings &settings)
    : robot_(settings.robot_ip, franka::RealtimeConfig::kEnforce) {
  // The same thresholds apply during acceleration and at nominal speed
  robot_.setCollisionBehavior(settings.lower_torque_thresholds, settings.upper_torque_thresholds,
                              settings.lower_torque_thresholds, settings.upper_torque_thresholds,
                              settings.lower_force_thresholds, settings.upper_force_thresholds,
                              settings.lower_force_thresholds, settings.upper_force_thresholds);
  robot_.setJointImpedance(settings.joint_impedance);
  robot_.setCartesianImpedance(settings.cartesian_impedance);
  robot_.setLoad(settings.load_mass, settings.F_x_Cload, settings.load_inertia);
  robot_.setEE(settings.NE_T_EE);
  model_.emplace(robot_.loadModel());
}

franka::RobotState FrankaRobotBackend::read_once() { return robot_.readOnce(); }

void FrankaRobotBackend::control_joint_positions(const JointPositionsCallback &callback, franka::ControllerMode mode,
                                                 bool limit_rate, double cutoff_frequency) {
  robot_.control(callback, mode, limit_rate, cutoff_frequency);
}

void FrankaRobotBackend::control_joint_velocities(const JointVelocitiesCallback &callback,
                                                  franka::ControllerMode mode, bool limit_rate,
                                                  double cutoff_frequency) {
  robot_.control(callback, mode, limit_rate, cutoff_frequency);
}

void FrankaRobotBackend::control_torques(const TorquesCallback &callback, bool limit_rate, double cutoff_frequency) {
  robot_.control(callback, limit_rate, cutoff_frequency);
}

void FrankaRobotBackend::control_cartesian_pose(const CartesianPoseCallback &callback, franka::ControllerMode mode,
                                                bool limit_rate, double cutoff_frequency) {
  robot_.control(callback, mode, limit_rate, cutoff_frequency);
}

void FrankaRobotBackend::control_cartesian_velocities(const CartesianVelocitiesCallback &callback,
                                                      franka::ControllerMode mode, bool limit_rate,
                                                      double cutoff_frequency) {
  robot_.control(callback, mode, limit_rate, cutoff_frequency);
}

void FrankaRobotBackend::automatic_error_recovery() { robot_.automaticErrorRecovery(); }

void FrankaRobotBackend::stop() { robot_.stop(); }

const franka::Model *FrankaRobotBackend::model() const { return model_ ? &*model_ : nullptr; }

FakeRobotBackend::FakeRobotBackend(const std::array<double, 7> &initial_q, double packet_loss,
                                   double tracking_offset)
    : packet_loss_(packet_loss), tracking_offset_(tracking_offset) {
  state_.q_d = initial_q;
  set_measured_from_commanded();
  // End effector half a meter above the base, well over any critical height
  state_.O_T_EE = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.3, 0.0, 0.5, 1};
  state_.O_T_EE_d = state_.O_T_EE_c = state_.O_T_EE;
  // Elbow: position of joint 3 and the side of joint 4, as libfranka defines it
  state_.elbow = {initial_q[2], initial_q[3] > -0.467002423653011 ? 1.0 : -1.0};
  state_.elbow_d = state_.elbow_c = state_.elbow;
  state_.robot_mode = franka::RobotMode::kIdle;
  state_.control_command_success_rate = 1.0;
}

franka::RobotState FakeRobotBackend::read_once() {
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  state_.time += franka::Duration(1);
  return state_;
}

void FakeRobotBackend::set_measured_from_commanded() {
  for (std::size_t i = 0; i < 7; ++i) {
    state_.q[i] = state_.q_d[i] + ((i % 2) ? tracking_offset_ : -tracking_offset_);
  }
  state_.dq = state_.dq_d;
}

template <typename Command, typename Apply>
void FakeRobotBackend::run(const std::function<Command(const franka::RobotState &, franka::Duration)> &callback,
                           Apply apply) {
  stop_requested_.store(false);
  state_.robot_mode = franka::RobotMode::kMove;
  franka::Duration period(0);
  auto next = std::chrono::steady_clock::now();
  while (true) {
    if (stop_requested_.load()) {
      state_.robot_mode = franka::RobotMode::kIdle;
      throw franka::ControlException("fake robot: motion stopped");
    }
    const Command command = callback(state_, period);
    // A lost packet: the callback is skipped once and the next one sees 2 ms
    const bool lost = packet_loss_ > 0.0 && std::rand() < packet_loss_ * RAND_MAX;
    const franka::Duration next_period(lost ? 2 : 1);
    apply(command, period, next_period);
    if (command.motion_finished) {
      state_.robot_mode = franka::RobotMode::kIdle;
      return;
    }
    next += std::chrono::milliseconds(next_period.toMSec());
    std::this_thread::sleep_until(next);
    period = next_period;
    state_.time += period;
  }
}

void FakeRobotBackend::control_joint_positions(const JointPositionsCallback &callback, franka::ControllerMode,
                                               bool, double) {
  run(callback, [this](const franka::JointPositions &command, franka::Duration period, franka::Duration) {
    for (std::size_t i = 0; i < 7; ++i) {
      state_.dq_d[i] = period.toMSec() > 0 ? (command.q[i] - state_.q_d[i]) / period.toSec() : 0.0;
    }
    state_.q_d = command.q;
    set_measured_from_commanded();
  });
}

void FakeRobotBackend::control_joint_velocities(const JointVelocitiesCallback &callback, franka::ControllerMode,
                                                bool, double) {
  run(callback, [this](const franka::JointVelocities &command, franka::Duration period, franka::Duration next) {
    for (std::size_t i = 0; i < 7; ++i) {
      state_.ddq_d[i] = period.toMSec() > 0 ? (command.dq[i] - state_.dq_d[i]) / period.toSec() : 0.0;
      state_.dq_d[i] = command.dq[i];
      // The command holds until the next one
      state_.q_d[i] += command.dq[i] * next.toSec();
    }
    set_measured_from_commanded();
  });
}

void FakeRobotBackend::control_torques(const TorquesCallback &callback, bool, double) {
  run(callback, [this](const franka::Torques &command, franka::Duration, franka::Duration next) {
    state_.tau_J_d = command.tau_J;
    for (uint64_t k = 0; k < next.toMSec(); ++k) {
      for (std::size_t i = 0; i < 7; ++i) {
        state_.dq[i] += command.tau_J[i] / kFakeInertia[i] * 1e-3;
        state_.q[i] += state_.dq[i] * 1e-3;
      }
    }
    state_.q_d = state_.q;
    state_.dq_d = state_.dq;
  });
}

void FakeRobotBackend::apply_elbow(const std::array<double, 2> &elbow, franka::Duration period) {
  const double dt = period.toSec();
  const double delbow = dt > 0.0 ? (elbow[0] - state_.elbow_c[0]) / dt : 0.0;
  state_.ddelbow_c[0] = dt > 0.0 ? (delbow - state_.delbow_c[0]) / dt : 0.0;
  state_.delbow_c[0] = delbow;
  state_.elbow = state_.elbow_d = state_.elbow_c = elbow;
}

void FakeRobotBackend::control_cartesian_pose(const CartesianPoseCallback &callback, franka::ControllerMode,
                                              bool, double) {
  run(callback, [this](const franka::CartesianPose &command, franka::Duration period, franka::Duration) {
    const double dt = period.toSec();
    const Pose last = pose_from_matrix(state_.O_T_EE_c);
    const Pose next = pose_from_matrix(command.O_T_EE);
    const Eigen::Vector3d rotation = rotation_between(last.q, next.q);
    for (std::size_t i = 0; i < 3; ++i) {
      const double v = dt > 0.0 ? (next.p[i] - last.p[i]) / dt : 0.0;
      const double w = dt > 0.0 ? rotation[i] / dt : 0.0;
      state_.O_ddP_EE_c[i] = dt > 0.0 ? (v - state_.O_dP_EE_c[i]) / dt : 0.0;
      state_.O_ddP_EE_c[3 + i] = dt > 0.0 ? (w - state_.O_dP_EE_c[3 + i]) / dt : 0.0;
      state_.O_dP_EE_c[i] = v;
      state_.O_dP_EE_c[3 + i] = w;
    }
    state_.O_T_EE = state_.O_T_EE_d = state_.O_T_EE_c = command.O_T_EE;
    if (command.hasElbow()) {
      apply_elbow(command.elbow, period);
    }
  });
}

void FakeRobotBackend::control_cartesian_velocities(const CartesianVelocitiesCallback &callback,
                                                    franka::ControllerMode, bool, double) {
  run(callback, [this](const franka::CartesianVelocities &command, franka::Duration period, franka::Duration next) {
    const double dt = period.toSec();
    for (std::size_t i = 0; i < 6; ++i) {
      state_.O_ddP_EE_c[i] = dt > 0.0 ? (command.O_dP_EE[i] - state_.O_dP_EE_c[i]) / dt : 0.0;
    }
    state_.O_dP_EE_c = command.O_dP_EE;
    // The command holds until the next one
    Pose pose = pose_from_matrix(state_.O_T_EE_c);
    const double hold = next.toSec();
    for (std::size_t i = 0; i < 3; ++i) {
      pose.p[i] += command.O_dP_EE[i] * hold;
    }
    pose.q = rotated(pose.q, Eigen::Vector3d(command.O_dP_EE[3], command.O_dP_EE[4], command.O_dP_EE[5]) * hold);
    state_.O_T_EE = state_.O_T_EE_d = state_.O_T_EE_c = matrix_from_pose(pose);
    if (command.hasElbow()) {
      apply_elbow(command.elbow, period);
    }
  });
}

}  // namespace panda_bridge
