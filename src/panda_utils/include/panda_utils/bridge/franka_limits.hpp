#pragma once

#include <franka/rate_limiting.h>

#include "panda_utils/bridge/types.hpp"
#include "panda_utils/robot_limits.hpp"

namespace panda_bridge {

// Kinematic limits of the FER arm. Position limits come from robot_limits.hpp, derivative limits from
// libfranka's rate_limiting.h, i.e. the same values the robot uses to detect discontinuities.
struct JointLimits {
  JointArray q_min{};
  JointArray q_max{};
  JointArray dq_max{};
  JointArray ddq_max{};
  JointArray dddq_max{};
};

inline JointLimits fer_joint_limits(double velocity_factor = 1.0, double acceleration_factor = 1.0,
                                    double jerk_factor = 1.0) {
  JointLimits limits;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    limits.q_min[i] = panda_constants::joint_min_limits[i];
    limits.q_max[i] = panda_constants::joint_max_limits[i];
    limits.dq_max[i] = franka::kMaxJointVelocity[i] * velocity_factor;
    limits.ddq_max[i] = franka::kMaxJointAcceleration[i] * acceleration_factor;
    limits.dddq_max[i] = franka::kMaxJointJerk[i] * jerk_factor;
  }
  return limits;
}

// Limits and gains of the torque bridge.
struct TorqueSettings {
  JointArray tau_max{};    // Nm
  JointArray rate_max{};   // Nm/s
  JointArray stiffness{};  // Nm/rad, internal motions and hold
  JointArray damping{};    // Nm s/rad, internal motions, hold and stops
};

// Effort limits of the FER and torque rate limit of libfranka, scaled by the factors; gains of the
// joint impedance example of libfranka.
inline TorqueSettings fer_torque_settings(double torque_factor = 1.0, double rate_factor = 1.0) {
  TorqueSettings settings;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    settings.tau_max[i] = panda_constants::effort_limits[i] * torque_factor;
    settings.rate_max[i] = franka::kMaxTorqueRate[i] * rate_factor;
  }
  settings.stiffness = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
  settings.damping = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
  return settings;
}

// Cartesian limits of the FER, as checked by the robot: norms of the translational and rotational
// velocity, acceleration and jerk of the end effector, and the elbow (joint 3).
struct CartesianLimits {
  double v_max{0.0}, a_max{0.0}, j_max{0.0};    // m/s, m/s^2, m/s^3
  double w_max{0.0}, dw_max{0.0}, ddw_max{0.0};  // rad/s, rad/s^2, rad/s^3
  double elbow_v_max{0.0}, elbow_a_max{0.0}, elbow_j_max{0.0};
  double elbow_min{0.0}, elbow_max{0.0};  // rad, joint 3 position limits
};

inline CartesianLimits fer_cartesian_limits(double velocity_factor = 1.0, double acceleration_factor = 1.0,
                                            double jerk_factor = 1.0) {
  CartesianLimits limits;
  limits.v_max = franka::kMaxTranslationalVelocity * velocity_factor;
  limits.a_max = franka::kMaxTranslationalAcceleration * acceleration_factor;
  limits.j_max = franka::kMaxTranslationalJerk * jerk_factor;
  // The pose interface allows a fraction of the rotational limits: use it for both interfaces
  const double rotation = franka::kFactorCartesianRotationPoseInterface;
  limits.w_max = rotation * franka::kMaxRotationalVelocity * velocity_factor;
  limits.dw_max = rotation * franka::kMaxRotationalAcceleration * acceleration_factor;
  limits.ddw_max = rotation * franka::kMaxRotationalJerk * jerk_factor;
  limits.elbow_v_max = franka::kMaxElbowVelocity * velocity_factor;
  limits.elbow_a_max = franka::kMaxElbowAcceleration * acceleration_factor;
  limits.elbow_j_max = franka::kMaxElbowJerk * jerk_factor;
  limits.elbow_min = panda_constants::joint_min_limits[2];
  limits.elbow_max = panda_constants::joint_max_limits[2];
  return limits;
}

}  // namespace panda_bridge
