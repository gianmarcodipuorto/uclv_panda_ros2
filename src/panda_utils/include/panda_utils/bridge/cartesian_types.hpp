#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Geometry>

#include "panda_utils/bridge/types.hpp"

namespace panda_bridge {

using Vec3 = std::array<double, 3>;
// Unit quaternion (x, y, z, w), the order of geometry_msgs
using Quat = std::array<double, 4>;

// Pose of the end effector in the base frame.
struct Pose {
  Vec3 p{};
  Quat q{0.0, 0.0, 0.0, 1.0};
};

// Axes of the Cartesian commands: vx, vy, vz, wx, wy, wz (base frame), elbow
constexpr std::size_t kCartesianAxes = 7;
using CartesianArray = std::array<double, kCartesianAxes>;

// State of the end effector and of the elbow (position of joint 3): commanded by the Cartesian
// trackers, or the reference of an internal motion at one instant.
struct CartesianState {
  Pose pose{};
  double elbow{0.0};
  CartesianArray velocity{};
  CartesianArray acceleration{};
};

// Cartesian quantities of the robot state used by the Cartesian pipelines: the last command.
struct CartesianFeedback {
  std::array<double, 16> O_T_EE_c{};   // pose, column major
  std::array<double, 6> O_dP_EE_c{};   // twist
  std::array<double, 6> O_ddP_EE_c{};  // twist derivative
  std::array<double, 2> elbow_c{};     // position of joint 3, sign of joint 4
  std::array<double, 2> delbow_c{};
  std::array<double, 2> ddelbow_c{};
};

// --- conversions and rotations; Eigen fixed-size types, no allocations ---

inline Eigen::Quaterniond to_eigen(const Quat &q) { return Eigen::Quaterniond(q[3], q[0], q[1], q[2]); }
inline Quat to_quat(const Eigen::Quaterniond &q) { return {q.x(), q.y(), q.z(), q.w()}; }
inline Eigen::Vector3d to_eigen(const Vec3 &v) { return Eigen::Vector3d(v[0], v[1], v[2]); }

inline Quat normalized(const Quat &q) {
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(norm > 0.0)) {
    return Quat{0.0, 0.0, 0.0, 1.0};
  }
  return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

// q or -q, whichever is closer to the reference: the same rotation, with a continuous sign
inline Quat aligned(const Quat &q, const Quat &reference) {
  const double dot = q[0] * reference[0] + q[1] * reference[1] + q[2] * reference[2] + q[3] * reference[3];
  return dot < 0.0 ? Quat{-q[0], -q[1], -q[2], -q[3]} : q;
}

// Rotation vector (axis times angle, angle in [0, pi]) of a unit quaternion.
inline Eigen::Vector3d rotation_vector(const Eigen::Quaterniond &q) {
  const Eigen::Quaterniond u = q.w() < 0.0 ? Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z()) : q;
  const double s = u.vec().norm();
  if (s < 1e-12) {
    return 2.0 * u.vec();
  }
  return u.vec() * (2.0 * std::atan2(s, u.w()) / s);
}

inline Eigen::Quaterniond from_rotation_vector(const Eigen::Vector3d &r) {
  const double angle = r.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(1.0, 0.5 * r.x(), 0.5 * r.y(), 0.5 * r.z()).normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, r / angle));
}

// Rotation vector r, in the base frame, such that to = exp(r) from. Divided by the cycle time it
// is the angular velocity the robot computes from two consecutive commanded orientations.
inline Eigen::Vector3d rotation_between(const Quat &from, const Quat &to) {
  return rotation_vector(to_eigen(to) * to_eigen(from).conjugate());
}

// Orientation q rotated by the rotation vector r, in the base frame.
inline Quat rotated(const Quat &q, const Eigen::Vector3d &r) {
  return to_quat((from_rotation_vector(r) * to_eigen(q)).normalized());
}

// Angular velocity (base frame) of an orientation q changing at rate q_dot: 2 q_dot q*.
inline Eigen::Vector3d angular_velocity(const Quat &q, const Quat &q_dot) {
  return 2.0 * (to_eigen(q_dot) * to_eigen(q).conjugate()).vec();
}

// Column-major homogeneous transform (franka O_T_EE) from a pose and back.
inline Pose pose_from_matrix(const std::array<double, 16> &m) {
  Eigen::Matrix3d rotation;
  rotation << m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10];
  Pose pose;
  pose.p = {m[12], m[13], m[14]};
  pose.q = to_quat(Eigen::Quaterniond(rotation).normalized());
  return pose;
}

inline std::array<double, 16> matrix_from_pose(const Pose &pose) {
  const Eigen::Matrix3d r = to_eigen(normalized(pose.q)).toRotationMatrix();
  return {r(0, 0), r(1, 0), r(2, 0), 0.0, r(0, 1), r(1, 1), r(2, 1), 0.0,
          r(0, 2), r(1, 2), r(2, 2), 0.0, pose.p[0], pose.p[1], pose.p[2], 1.0};
}

// Norm of the three axes of a Cartesian array starting at first (0 translation, 3 rotation).
inline double norm3(const CartesianArray &values, std::size_t first) {
  return std::sqrt(values[first] * values[first] + values[first + 1] * values[first + 1] +
                   values[first + 2] * values[first + 2]);
}

}  // namespace panda_bridge
