#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <ruckig/ruckig.hpp>

#include "panda_utils/bridge/franka_limits.hpp"
#include "panda_utils/bridge/joint_motions.hpp"
#include "panda_utils/bridge/pipeline_types.hpp"

namespace panda_bridge {

// Jerk-limited follower of the commanded joint positions. The robot checks the commands with
// finite differences (velocity, acceleration, jerk of consecutive samples), so the tracker does
// the same: a reference sample that passes the check is commanded as it is, reproducing a
// feasible reference exactly. Otherwise the command converges to the reference through a
// Ruckig velocity-interface step: target velocity = reference velocity + position correction,
// linear near the reference and braking-limited (square root) far from it. Velocity targets never
// force the command to reach an exact state, so there are no overshoot-and-return loops.
// Joint limits are enforced by bounding the velocity with the braking distance.
// Fixed number of DoFs: Ruckig does not allocate memory in update().
class JointTracker {
 public:
  // Types and properties used by Pipeline: the stream is played back with the smooth fit
  using Values = JointArray;
  using Feedback = JointFeedback;
  using Motion = JointMotion;
  using MotionPoint = JointPoint;
  using Command = JointArray;
  using Position = JointArray;
  static constexpr bool kSmoothStream = true;
  static constexpr bool kStreamOfVelocities = false;
  static constexpr bool kSupportsFreeMotion = false;

  explicit JointTracker(double cycle_time = kCycleTime) : otg_(cycle_time), dt_(cycle_time) {
    input_.control_interface = ruckig::ControlInterface::Velocity;
    input_.synchronization = ruckig::Synchronization::None;
  }

  // position_margin keeps the command this far from the joint limits.
  void set_limits(const JointLimits &limits, double position_margin) {
    limits_ = limits;
    margin_ = position_margin;
  }

  const JointLimits &limits() const { return limits_; }

  void reset(const JointArray &p, const JointArray &v, const JointArray &a) {
    p_ = p;
    v_ = v;
    a_ = a;
    // Previous commands consistent with the initial state
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      h0_[i] = p[i];
      h1_[i] = p[i] - v[i] * dt_ + 0.5 * a[i] * dt_ * dt_;
      h2_[i] = p[i] - 2 * v[i] * dt_ + 2 * a[i] * dt_ * dt_;
    }
    direct_ = true;
  }

  // One cycle towards the reference state. Returns true if the reference could not be commanded
  // as it is (limits, or convergence after a deviation).
  bool track(const JointArray &p_ref, const JointArray &v_ref, const JointArray &a_ref) {
    update_velocity_bounds();
    JointArray target = p_ref;
    bool clamped = false;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      target[i] = std::clamp(p_ref[i], limits_.q_min[i] + margin_, limits_.q_max[i] - margin_);
      clamped = clamped || target[i] != p_ref[i];
    }
    // Back from a deviation only once converged: a candidate needing little jerk. Taking it at
    // the full jerk limit would leave a velocity step behind and start an oscillation.
    if (!clamped && feasible(target, direct_ ? 1.0 : kReentryJerkFraction)) {
      accept_sample(target);
      direct_ = true;
      return false;
    }
    direct_ = false;

    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const bool beyond = target[i] != p_ref[i];
      const double error = target[i] - p_[i];
      const double a_corr = 0.5 * limits_.ddq_max[i];
      const double correction =
          std::copysign(std::min(kPositionGain * std::abs(error), std::sqrt(2 * a_corr * std::abs(error))), error);
      // No feedforward towards a reference beyond the joint limits
      const double v_ff = beyond ? 0.0 : v_ref[i];
      const double a_ff = beyond ? 0.0 : a_ref[i];
      input_.target_velocity[i] = std::clamp(v_ff + correction, -v_lower_[i], v_upper_[i]);
      input_.target_acceleration[i] = std::clamp(a_ff, -limits_.ddq_max[i], limits_.ddq_max[i]);
    }
    step_velocity_interface();
    return true;
  }

  // One cycle of maximal braking towards zero velocity and acceleration. Returns true at rest.
  bool brake() {
    input_.target_velocity.fill(0.0);
    input_.target_acceleration.fill(0.0);
    step_velocity_interface();
    if (at_rest()) {
      hold();
      return true;
    }
    return false;
  }

  // Keeps the current position. Only meaningful when already at rest.
  void hold() {
    v_.fill(0.0);
    a_.fill(0.0);
    h2_ = h1_ = h0_ = p_;
    direct_ = true;
  }

  bool at_rest() const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (std::abs(v_[i]) > kRestVelocity || std::abs(a_[i]) > kRestAcceleration) {
        return false;
      }
    }
    return true;
  }

  const JointArray &position() const { return p_; }
  const JointArray &velocity() const { return v_; }
  const JointArray &acceleration() const { return a_; }
  uint64_t errors() const { return errors_; }

  // --- interface used by Pipeline ---
  void configure(const PipelineTypes::Config &config) {
    set_limits(config.limits, config.position_margin);
    max_jump_ = config.max_jump;
  }
  void reset(const JointFeedback &robot) { reset(robot.q_d, robot.dq_d, robot.ddq_d); }
  void set_feedback(const JointFeedback &) {}
  void align_sample(JointArray &, const JointArray *) const {}
  // A stream that (re)starts farther than max_jump from the command is rejected
  bool rejects_stream_start(const JointArray &p, double &distance) const {
    distance = 0.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      distance = std::max(distance, std::abs(p[i] - p_[i]));
    }
    return distance > max_jump_;
  }
  bool follow_stream(const JointArray &p, const JointArray &v, const JointArray &a) { return track(p, v, a); }
  bool prepare_motion(JointMotion &motion) const {
    return prepare_joint_motion(motion, p_, limits_, margin_, kSupportsFreeMotion);
  }
  bool follow_motion(const JointArray &p, const JointArray &v, const JointArray &a) { return track(p, v, a); }
  bool follow_motion(const JointPoint &point) { return track(point.p, point.v, point.a); }
  // For clamp disabled: a stream reference beyond the limits stops the robot
  bool reference_within_limits(const JointArray &p, const JointArray &v) const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (p[i] < limits_.q_min[i] + margin_ || p[i] > limits_.q_max[i] - margin_ ||
          std::abs(v[i]) > limits_.dq_max[i]) {
        return false;
      }
    }
    return true;
  }
  bool nearly_at_rest() const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (std::abs(v_[i]) > 1e-3 || std::abs(a_[i]) > 1e-2) {
        return false;
      }
    }
    return true;
  }
  // Command sent to the robot
  const JointArray &command() const { return p_; }

 private:
  static constexpr double kPositionGain = 30.0;  // 1/s, convergence after a deviation
  // A snap at up to this fraction of the jerk limit is undone within the limit by the next samples
  static constexpr double kReentryJerkFraction = 0.3;
  static constexpr double kRestVelocity = 1e-5;
  static constexpr double kRestAcceleration = 1e-4;

  // Velocity allowed towards each joint limit, so that braking at half the acceleration limit
  // stops before the limit (the other half covers the jerk-limited ramp).
  void update_velocity_bounds() {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double a_brake = 0.5 * limits_.ddq_max[i];
      const double to_upper = std::max(0.0, limits_.q_max[i] - margin_ - p_[i]);
      const double to_lower = std::max(0.0, p_[i] - (limits_.q_min[i] + margin_));
      v_upper_[i] = std::min(limits_.dq_max[i], std::sqrt(2 * a_brake * to_upper));
      v_lower_[i] = std::min(limits_.dq_max[i], std::sqrt(2 * a_brake * to_lower));
    }
  }

  // Finite differences of the candidate against the last commands, as checked by the robot.
  bool feasible(const JointArray &candidate, double jerk_fraction) const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double v = (candidate[i] - h0_[i]) / dt_;
      const double v0 = (h0_[i] - h1_[i]) / dt_;
      const double v1 = (h1_[i] - h2_[i]) / dt_;
      const double a = (v - v0) / dt_;
      const double a0 = (v0 - v1) / dt_;
      const double j = (a - a0) / dt_;
      if (v > v_upper_[i] || v < -v_lower_[i] || std::abs(a) > limits_.ddq_max[i] ||
          std::abs(j) > jerk_fraction * limits_.dddq_max[i]) {
        return false;
      }
    }
    return true;
  }

  void accept_sample(const JointArray &p) {
    h2_ = h1_;
    h1_ = h0_;
    h0_ = p;
    p_ = p;
    // Continuous state consistent with the commanded samples: the backward difference is the
    // velocity half a cycle ago, the second difference the acceleration one cycle ago
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double v0 = (h0_[i] - h1_[i]) / dt_;
      const double a0 = (v0 - (h1_[i] - h2_[i]) / dt_) / dt_;
      v_[i] = v0 + 0.5 * a0 * dt_;
      a_[i] = a0;
    }
  }

  void step_velocity_interface() {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      input_.current_position[i] = p_[i];
      input_.current_velocity[i] = v_[i];
      input_.current_acceleration[i] = a_[i];
      input_.max_velocity[i] = limits_.dq_max[i];
      input_.max_acceleration[i] = limits_.ddq_max[i];
      input_.max_jerk[i] = limits_.dddq_max[i];
    }
    if (otg_.update(input_, output_) < 0) {
      // Should not happen with valid limits: freeze rather than command an undefined state
      ++errors_;
      hold();
      return;
    }
    h2_ = h1_;
    h1_ = h0_;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      p_[i] = output_.new_position[i];
      v_[i] = output_.new_velocity[i];
      a_[i] = output_.new_acceleration[i];
    }
    h0_ = p_;
  }

  ruckig::Ruckig<kNumJoints> otg_;
  double dt_;
  ruckig::InputParameter<kNumJoints> input_;
  ruckig::OutputParameter<kNumJoints> output_;
  JointLimits limits_{fer_joint_limits()};
  double margin_{0.0};
  double max_jump_{0.2};
  JointArray p_{}, v_{}, a_{};
  // Last three commanded positions, newest first
  JointArray h0_{}, h1_{}, h2_{};
  JointArray v_upper_{}, v_lower_{};
  // True while the reference is commanded as it is
  bool direct_{true};
  uint64_t errors_{0};
};

}  // namespace panda_bridge
