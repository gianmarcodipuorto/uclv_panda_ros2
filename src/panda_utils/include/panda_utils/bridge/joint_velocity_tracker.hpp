#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <ruckig/ruckig.hpp>

#include "panda_utils/bridge/franka_limits.hpp"
#include "panda_utils/bridge/joint_motions.hpp"
#include "panda_utils/bridge/pipeline_types.hpp"

namespace panda_bridge {

// Jerk-limited follower of the commanded joint velocities (franka::JointVelocities). The robot
// checks velocity commands with finite differences (acceleration and jerk of consecutive
// samples), so the tracker does the same: a reference sample that passes the check is commanded
// as it is. Otherwise the command converges to the reference velocity through a Ruckig
// velocity-interface step. The commands are integrated as the robot integrates them, and the
// velocity is bounded by the braking distance to the joint limits.
// Fixed number of DoFs: Ruckig does not allocate memory in update().
class JointVelocityTracker {
 public:
  // Types and properties used by Pipeline: the stream is made of velocities, played back with the
  // smooth fit
  using Values = JointArray;
  using Feedback = JointFeedback;
  using Motion = JointMotion;
  using MotionPoint = JointPoint;
  using Command = JointArray;
  using Position = JointArray;
  static constexpr bool kSmoothStream = true;
  static constexpr bool kStreamOfVelocities = true;
  static constexpr bool kSupportsFreeMotion = false;

  explicit JointVelocityTracker(double cycle_time = kCycleTime) : otg_(cycle_time), dt_(cycle_time) {
    input_.control_interface = ruckig::ControlInterface::Velocity;
    input_.synchronization = ruckig::Synchronization::None;
  }

  // position_margin keeps the integrated command this far from the joint limits.
  void set_limits(const JointLimits &limits, double position_margin) {
    limits_ = limits;
    margin_ = position_margin;
  }

  const JointLimits &limits() const { return limits_; }

  // p: commanded position (q_d); v, a: commanded velocity and acceleration.
  void reset(const JointArray &p, const JointArray &v, const JointArray &a) {
    p_ = p;
    v_ = v;
    a_ = a;
    // Previous commands consistent with the initial state
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      u0_[i] = v[i];
      u1_[i] = v[i] - a[i] * dt_;
    }
    direct_ = true;
  }

  // One cycle towards the reference velocity. Returns true if the reference could not be
  // commanded as it is (limits, or convergence after a deviation).
  bool track(const JointArray &v_ref, const JointArray &a_ref) {
    update_velocity_bounds();
    JointArray target = v_ref;
    bool clamped = false;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      target[i] = std::clamp(v_ref[i], -v_lower_[i], v_upper_[i]);
      clamped = clamped || target[i] != v_ref[i];
    }
    // Back from a deviation only once converged, as in JointTracker
    if (!clamped && feasible(target, direct_ ? 1.0 : kReentryJerkFraction)) {
      accept_sample(target);
      direct_ = true;
      return false;
    }
    direct_ = false;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      input_.target_velocity[i] = target[i];
      // No feedforward towards a clamped velocity
      input_.target_acceleration[i] =
          target[i] != v_ref[i] ? 0.0 : std::clamp(a_ref[i], -limits_.ddq_max[i], limits_.ddq_max[i]);
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

  // Zero velocity. Only meaningful when already at rest.
  void hold() {
    v_.fill(0.0);
    a_.fill(0.0);
    u0_.fill(0.0);
    u1_.fill(0.0);
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

  // Integral of the commanded velocities, from the commanded position at reset
  const JointArray &position() const { return p_; }
  const JointArray &velocity() const { return v_; }
  const JointArray &acceleration() const { return a_; }
  uint64_t errors() const { return errors_; }

  // --- interface used by Pipeline ---
  void configure(const PipelineTypes::Config &config) { set_limits(config.limits, config.position_margin); }
  void reset(const JointFeedback &robot) { reset(robot.q_d, robot.dq_d, robot.ddq_d); }
  void set_feedback(const JointFeedback &) {}
  void align_sample(JointArray &, const JointArray *) const {}
  bool rejects_stream_start(const JointArray &, double &distance) const {
    distance = 0.0;
    return false;
  }
  bool follow_stream(const JointArray &v, const JointArray &a, const JointArray &) { return track(v, a); }
  bool prepare_motion(JointMotion &motion) const {
    return prepare_joint_motion(motion, p_, limits_, margin_, kSupportsFreeMotion);
  }
  bool follow_motion(const JointPoint &point) { return follow_motion(point.p, point.v, point.a); }
  // Internal motions are positions: their velocity, plus a correction that keeps the integrated
  // command on the reference position (linear near it, braking-limited far from it)
  bool follow_motion(const JointArray &p, const JointArray &v, const JointArray &a) {
    JointArray v_ref;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double error = p[i] - p_[i];
      const double a_corr = 0.5 * limits_.ddq_max[i];
      v_ref[i] = v[i] + std::copysign(std::min(kPositionGain * std::abs(error), std::sqrt(2 * a_corr * std::abs(error))),
                                      error);
    }
    return track(v_ref, a);
  }
  // For clamp disabled: a stream reference beyond the limits stops the robot
  bool reference_within_limits(const JointArray &v, const JointArray &) const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (std::abs(v[i]) > limits_.dq_max[i]) {
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
  const JointArray &command() const { return v_; }

 private:
  static constexpr double kPositionGain = 30.0;  // 1/s, approach of positions and joint limits
  // A snap at up to this fraction of the jerk limit is undone within the limit by the next samples
  static constexpr double kReentryJerkFraction = 0.3;
  static constexpr double kRestVelocity = 1e-5;
  static constexpr double kRestAcceleration = 1e-4;

  // Velocity allowed towards each joint limit, so that braking at half the acceleration limit
  // stops before the limit (the other half covers the jerk-limited ramp). Close to the limit the
  // bound becomes linear in the distance: the joint approaches it exponentially, without the
  // overshoot of a jerk-limited stop at the end of the braking parabola.
  void update_velocity_bounds() {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double a_brake = 0.5 * limits_.ddq_max[i];
      const double to_upper = std::max(0.0, limits_.q_max[i] - margin_ - p_[i]);
      const double to_lower = std::max(0.0, p_[i] - (limits_.q_min[i] + margin_));
      v_upper_[i] = std::min({limits_.dq_max[i], std::sqrt(2 * a_brake * to_upper), kPositionGain * to_upper});
      v_lower_[i] = std::min({limits_.dq_max[i], std::sqrt(2 * a_brake * to_lower), kPositionGain * to_lower});
    }
  }

  // Finite differences of the candidate against the last commands, as checked by the robot.
  bool feasible(const JointArray &candidate, double jerk_fraction) const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double a = (candidate[i] - u0_[i]) / dt_;
      const double a0 = (u0_[i] - u1_[i]) / dt_;
      const double j = (a - a0) / dt_;
      if (candidate[i] > v_upper_[i] || candidate[i] < -v_lower_[i] || std::abs(a) > limits_.ddq_max[i] ||
          std::abs(j) > jerk_fraction * limits_.dddq_max[i]) {
        return false;
      }
    }
    return true;
  }

  void accept_sample(const JointArray &v) {
    u1_ = u0_;
    u0_ = v;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      a_[i] = (u0_[i] - u1_[i]) / dt_;
      p_[i] += v[i] * dt_;
    }
    v_ = v;
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
      // Should not happen with valid limits: keep the velocity rather than command an undefined state
      ++errors_;
      accept_sample(v_);
      return;
    }
    u1_ = u0_;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      v_[i] = output_.new_velocity[i];
      a_[i] = output_.new_acceleration[i];
      // Integrated as the robot does, one command per cycle
      p_[i] += v_[i] * dt_;
    }
    u0_ = v_;
  }

  ruckig::Ruckig<kNumJoints> otg_;
  double dt_;
  ruckig::InputParameter<kNumJoints> input_;
  ruckig::OutputParameter<kNumJoints> output_;
  JointLimits limits_{fer_joint_limits()};
  double margin_{0.0};
  JointArray p_{}, v_{}, a_{};
  // Last two commanded velocities, newest first
  JointArray u0_{}, u1_{};
  JointArray v_upper_{}, v_lower_{};
  // True while the reference is commanded as it is
  bool direct_{true};
  uint64_t errors_{0};
};

}  // namespace panda_bridge
