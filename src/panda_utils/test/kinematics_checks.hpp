#pragma once

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "panda_utils/bridge/cartesian_types.hpp"
#include "panda_utils/bridge/franka_limits.hpp"

namespace panda_bridge::test {

// Cartesian commands checked the way the robot does: the twist of consecutive commanded poses
// (or the commanded twist itself), then norms of velocity, acceleration and jerk of translation,
// rotation and elbow.
inline void expect_cartesian_within_limits(const std::vector<CartesianState> &commands, const CartesianLimits &limits,
                                           bool pose_commands, double tolerance = 1e-6) {
  const double dt = kCycleTime;
  std::vector<CartesianArray> velocities;
  for (std::size_t k = 1; k < commands.size(); ++k) {
    CartesianArray u = commands[k].velocity;
    if (pose_commands) {
      const Eigen::Vector3d r = rotation_between(commands[k - 1].pose.q, commands[k].pose.q);
      for (std::size_t i = 0; i < 3; ++i) {
        u[i] = (commands[k].pose.p[i] - commands[k - 1].pose.p[i]) / dt;
        u[3 + i] = r[i] / dt;
      }
      u[6] = (commands[k].elbow - commands[k - 1].elbow) / dt;
    }
    velocities.push_back(u);
  }
  for (std::size_t k = 2; k < velocities.size(); ++k) {
    CartesianArray a{}, j{};
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      a[i] = (velocities[k][i] - velocities[k - 1][i]) / dt;
      j[i] = (velocities[k][i] - 2 * velocities[k - 1][i] + velocities[k - 2][i]) / (dt * dt);
    }
    const auto &u = velocities[k];
    ASSERT_LE(norm3(u, 0), limits.v_max * (1 + tolerance) + 1e-9) << "cycle " << k;
    ASSERT_LE(norm3(u, 3), limits.w_max * (1 + tolerance) + 1e-9) << "cycle " << k;
    ASSERT_LE(std::abs(u[6]), limits.elbow_v_max * (1 + tolerance) + 1e-9) << "cycle " << k;
    ASSERT_LE(norm3(a, 0), limits.a_max * (1 + tolerance) + 1e-6) << "cycle " << k;
    ASSERT_LE(norm3(a, 3), limits.dw_max * (1 + tolerance) + 1e-6) << "cycle " << k;
    ASSERT_LE(std::abs(a[6]), limits.elbow_a_max * (1 + tolerance) + 1e-6) << "cycle " << k;
    ASSERT_LE(norm3(j, 0), limits.j_max * (1 + tolerance) + 1e-3) << "cycle " << k;
    ASSERT_LE(norm3(j, 3), limits.ddw_max * (1 + tolerance) + 1e-3) << "cycle " << k;
    ASSERT_LE(std::abs(j[6]), limits.elbow_j_max * (1 + tolerance) + 1e-3) << "cycle " << k;
  }
}

// End effector pointing down, half a meter in front of the base: a comfortable pose of the FER.
inline Pose ready_ee_pose() {
  Pose pose;
  pose.p = {0.4, 0.0, 0.4};
  pose.q = {1.0, 0.0, 0.0, 0.0};  // 180 degrees about x
  return pose;
}

// Checks a sequence of commands sampled at 1 kHz the way the robot does: finite differences of
// velocity, acceleration and jerk against the limits. For a trajectory whose derivatives are
// bounded, each divided difference equals the derivative at some intermediate time, so this check
// is exact up to rounding.
inline void expect_within_limits(const std::vector<JointArray> &commands, const JointLimits &limits,
                                 double tolerance = 1e-6) {
  const double dt = kCycleTime;
  for (std::size_t k = 3; k < commands.size(); ++k) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double p0 = commands[k - 3][i], p1 = commands[k - 2][i];
      const double p2 = commands[k - 1][i], p3 = commands[k][i];
      const double v = (p3 - p2) / dt;
      const double a = (p3 - 2 * p2 + p1) / (dt * dt);
      const double j = (p3 - 3 * p2 + 3 * p1 - p0) / (dt * dt * dt);
      ASSERT_LE(std::abs(v), limits.dq_max[i] * (1 + tolerance) + 1e-9) << "joint " << i << " cycle " << k;
      ASSERT_LE(std::abs(a), limits.ddq_max[i] * (1 + tolerance) + 1e-6) << "joint " << i << " cycle " << k;
      ASSERT_LE(std::abs(j), limits.dddq_max[i] * (1 + tolerance) + 1e-3) << "joint " << i << " cycle " << k;
      ASSERT_GE(p3, limits.q_min[i]) << "joint " << i << " cycle " << k;
      ASSERT_LE(p3, limits.q_max[i]) << "joint " << i << " cycle " << k;
    }
  }
}

// Same check for a sequence of velocity commands: acceleration and jerk are their first and second
// differences.
inline void expect_velocities_within_limits(const std::vector<JointArray> &commands, const JointLimits &limits,
                                            double tolerance = 1e-6) {
  const double dt = kCycleTime;
  for (std::size_t k = 2; k < commands.size(); ++k) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double v0 = commands[k - 2][i], v1 = commands[k - 1][i], v2 = commands[k][i];
      const double a = (v2 - v1) / dt;
      const double j = (v2 - 2 * v1 + v0) / (dt * dt);
      ASSERT_LE(std::abs(v2), limits.dq_max[i] * (1 + tolerance) + 1e-9) << "joint " << i << " cycle " << k;
      ASSERT_LE(std::abs(a), limits.ddq_max[i] * (1 + tolerance) + 1e-6) << "joint " << i << " cycle " << k;
      ASSERT_LE(std::abs(j), limits.dddq_max[i] * (1 + tolerance) + 1e-3) << "joint " << i << " cycle " << k;
    }
  }
}

// A comfortable configuration of the FER arm, far from the joint limits.
inline JointArray ready_pose() { return {0.0, -M_PI_4, 0.0, -3 * M_PI_4, 0.0, M_PI_2, M_PI_4}; }

}  // namespace panda_bridge::test
