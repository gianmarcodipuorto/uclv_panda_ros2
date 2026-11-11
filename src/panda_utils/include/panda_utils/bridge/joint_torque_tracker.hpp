#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "panda_utils/bridge/franka_limits.hpp"
#include "panda_utils/bridge/joint_motions.hpp"
#include "panda_utils/bridge/pipeline_types.hpp"

namespace panda_bridge {

// Commanded joint torques (franka::Torques, without gravity: the robot compensates it). Every
// command is clamped to the effort limits and rate limited, because the robot rejects torque
// discontinuities: neither the stream nor a change of mode can produce a torque step.
//   stream : the reference torques, as they are when within the limits
//   motion : joint impedance on the reference, K (q_ref - q) + D (dq_ref - dq) + coriolis
//   free   : coriolis only, i.e. gravity compensation: the robot can be moved by hand
//   brake  : damping -D dq + coriolis, until the joints are at rest
//   hold   : joint impedance at the last motion reference, or where the robot came to rest
// Unlike the position and velocity trackers this is a feedback law: set_feedback() before
// every cycle.
class JointTorqueTracker {
 public:
  // Types and properties used by Pipeline: the stream is made of torques, played back linearly
  using Values = JointArray;
  using Feedback = JointFeedback;
  using Motion = JointMotion;
  using MotionPoint = JointPoint;
  using Command = JointArray;
  using Position = JointArray;
  static constexpr bool kSmoothStream = false;
  static constexpr bool kSupportsFreeMotion = true;

  explicit JointTorqueTracker(double cycle_time = kCycleTime) : dt_(cycle_time) {}

  // Kinematic limits are used for the internal motions only.
  void set_limits(const JointLimits &limits, double position_margin) {
    limits_ = limits;
    margin_ = position_margin;
  }
  void set_torque_settings(const TorqueSettings &settings) { settings_ = settings; }

  const JointLimits &limits() const { return limits_; }
  const TorqueSettings &torque_settings() const { return settings_; }

  void configure(const PipelineTypes::Config &config) {
    set_limits(config.limits, config.position_margin);
    set_torque_settings(config.torque);
  }

  void reset(const JointFeedback &robot) {
    set_feedback(robot);
    // Rate limited from the torque the robot is applying
    tau_ = robot.tau_d;
    hold_valid_ = false;
    hold_ref_ = robot.q;
  }

  void set_feedback(const JointFeedback &robot) {
    q_ = robot.q;
    dq_ = robot.dq;
    coriolis_ = robot.coriolis;
  }

  void align_sample(JointArray &, const JointArray *) const {}

  bool rejects_stream_start(const JointArray &, double &distance) const {
    distance = 0.0;
    return false;
  }

  bool follow_stream(const JointArray &tau, const JointArray &, const JointArray &) {
    hold_valid_ = false;
    return command(tau);
  }

  bool prepare_motion(JointMotion &motion) const {
    return prepare_joint_motion(motion, position(), limits_, margin_, kSupportsFreeMotion);
  }

  bool follow_motion(const JointPoint &point) { return follow_motion(point.p, point.v, point.a); }

  bool follow_motion(const JointArray &p, const JointArray &v, const JointArray &) {
    // A hold after the motion keeps its last reference
    hold_ref_ = p;
    hold_valid_ = true;
    JointArray tau;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      tau[i] = settings_.stiffness[i] * (p[i] - q_[i]) + settings_.damping[i] * (v[i] - dq_[i]) + coriolis_[i];
    }
    return command(tau);
  }

  bool follow_free() {
    hold_valid_ = false;
    return command(coriolis_);
  }

  // One cycle of damping. Returns true once the joints are at rest.
  bool brake() {
    hold_valid_ = false;
    JointArray tau;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      tau[i] = -settings_.damping[i] * dq_[i] + coriolis_[i];
    }
    command(tau);
    return at_rest();
  }

  void hold() {
    if (!hold_valid_) {
      hold_ref_ = q_;
      hold_valid_ = true;
    }
    JointArray tau;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      tau[i] = settings_.stiffness[i] * (hold_ref_[i] - q_[i]) - settings_.damping[i] * dq_[i] + coriolis_[i];
    }
    command(tau);
  }

  // Measured joint speeds below kRestSpeed
  bool at_rest() const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (std::abs(dq_[i]) > kRestSpeed) {
        return false;
      }
    }
    return true;
  }
  bool nearly_at_rest() const { return at_rest(); }

  // For clamp disabled: a stream reference beyond the limits stops the robot
  bool reference_within_limits(const JointArray &tau, const JointArray &) const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (std::abs(tau[i]) > settings_.tau_max[i]) {
        return false;
      }
    }
    return true;
  }

  // Position held, or the measured one: internal motions start from here
  const JointArray &position() const { return hold_valid_ ? hold_ref_ : q_; }
  // Command sent to the robot
  const JointArray &command() const { return tau_; }
  uint64_t errors() const { return 0; }

 private:
  static constexpr double kRestSpeed = 0.02;  // rad/s

  // Clamps to the effort limits and limits the rate. Returns true if the target was changed.
  bool command(const JointArray &target) {
    bool limited = false;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double step = settings_.rate_max[i] * dt_;
      const double clamped = std::clamp(target[i], -settings_.tau_max[i], settings_.tau_max[i]);
      tau_[i] = std::clamp(clamped, tau_[i] - step, tau_[i] + step);
      limited = limited || tau_[i] != target[i];
    }
    return limited;
  }

  double dt_;
  JointLimits limits_{fer_joint_limits()};
  double margin_{0.0};
  TorqueSettings settings_{fer_torque_settings()};
  JointArray q_{}, dq_{}, coriolis_{};
  JointArray tau_{};
  JointArray hold_ref_{};
  bool hold_valid_{false};
};

}  // namespace panda_bridge
