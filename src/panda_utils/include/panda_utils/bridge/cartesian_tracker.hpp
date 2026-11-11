#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include <ruckig/ruckig.hpp>

#include "panda_utils/bridge/cartesian_motions.hpp"
#include "panda_utils/bridge/cartesian_types.hpp"
#include "panda_utils/bridge/pipeline_types.hpp"

namespace panda_bridge {

// Jerk-limited follower of the Cartesian commands: end effector poses (franka::CartesianPose) or
// twists (franka::CartesianVelocities), plus the elbow when it is commanded.
// The robot computes the twist from consecutive commanded poses and checks the norms of the
// translational and rotational velocity, acceleration and jerk, and those of the elbow; the
// tracker does the same. A reference that passes the check is commanded as it is, so a feasible
// stream is reproduced exactly. Otherwise the command converges to the reference through a Ruckig
// velocity-interface step on the 7 axes, with per-axis limits of 1/sqrt(3) of the norms so that
// the norms stay within the limits; poses converge with a correction of the pose error, linear
// near the reference and braking-limited far from it (as JointTracker).
// The commanded end effector never goes below min_height: the downward speed is bounded by the
// braking distance, as the joints are bounded near their limits. The elbow likewise.
// Fixed number of DoFs: Ruckig does not allocate memory in update().
template <bool kPoseCommands>
class CartesianTracker {
 public:
  // Types and properties used by Pipeline. Stream samples are poses (x, y, z, qx, qy, qz, qw) or
  // twists (vx, vy, vz, wx, wy, wz) in the base frame, played back with the smooth fit
  using Values = std::conditional_t<kPoseCommands, std::array<double, 7>, std::array<double, 6>>;
  using Feedback = CartesianFeedback;
  using Motion = CartesianMotion;
  using MotionPoint = CartesianState;
  using Command = CartesianState;
  using Position = CartesianState;
  static constexpr bool kSmoothStream = true;
  static constexpr bool kStreamOfVelocities = !kPoseCommands;
  static constexpr bool kSupportsFreeMotion = false;

  explicit CartesianTracker(double cycle_time = kCycleTime) : otg_(cycle_time), dt_(cycle_time) {
    input_.control_interface = ruckig::ControlInterface::Velocity;
    input_.synchronization = ruckig::Synchronization::None;
    set_settings(CartesianSettings{});
  }

  void set_settings(const CartesianSettings &settings) {
    settings_ = settings;
    const CartesianLimits &l = settings.limits;
    const double per_axis = 1.0 / std::sqrt(3.0);
    for (std::size_t i = 0; i < 3; ++i) {
      v_axis_[i] = l.v_max;
      a_axis_[i] = l.a_max * per_axis;
      j_axis_[i] = l.j_max * per_axis;
      v_axis_[3 + i] = l.w_max;
      a_axis_[3 + i] = l.dw_max * per_axis;
      j_axis_[3 + i] = l.ddw_max * per_axis;
    }
    v_axis_[6] = l.elbow_v_max;
    a_axis_[6] = l.elbow_a_max;
    j_axis_[6] = l.elbow_j_max;
  }
  const CartesianSettings &settings() const { return settings_; }

  // From the last command reported by the robot
  void reset(const CartesianFeedback &robot) {
    state_ = CartesianState{};
    state_.pose = pose_from_matrix(robot.O_T_EE_c);
    state_.elbow = robot.elbow_c[0];
    for (std::size_t i = 0; i < 6; ++i) {
      state_.velocity[i] = robot.O_dP_EE_c[i];
      state_.acceleration[i] = robot.O_ddP_EE_c[i];
    }
    if (settings_.use_elbow) {
      state_.velocity[6] = robot.delbow_c[0];
      state_.acceleration[6] = robot.ddelbow_c[0];
    }
    // Previous commands consistent with the initial state
    u0_ = state_.velocity;
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      u1_[i] = state_.velocity[i] - state_.acceleration[i] * dt_;
    }
    direct_ = true;
  }

  // Pose commands: one cycle towards the reference state. Returns true if the reference could not
  // be commanded as it is (limits, or convergence after a deviation).
  bool track_pose(const CartesianState &ref) {
    update_bounds();
    Pose target = ref.pose;
    double elbow = settings_.use_elbow ? ref.elbow : state_.elbow;
    const bool clamped_height = target.p[2] < settings_.min_height;
    target.p[2] = std::max(target.p[2], settings_.min_height);
    const double elbow_clamped = std::clamp(elbow, elbow_min(), elbow_max());
    const bool clamped_elbow = settings_.use_elbow && elbow_clamped != elbow;
    if (settings_.use_elbow) {
      elbow = elbow_clamped;
    }
    // Velocity of the candidate as the robot computes it from consecutive poses
    const CartesianArray u = velocity_to(target, elbow);
    // Back from a deviation only once converged, as in JointTracker
    if (!clamped_height && !clamped_elbow && feasible(u, direct_ ? 1.0 : kReentryJerkFraction)) {
      accept(u);
      state_.pose = target;
      state_.elbow = elbow;
      direct_ = true;
      return false;
    }
    direct_ = false;
    CartesianArray velocity = correction_towards(target, elbow);
    CartesianArray acceleration{};
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      // No feedforward towards a reference beyond the limits
      const bool beyond = (i == 2 && clamped_height) || (i == 6 && clamped_elbow);
      if (!beyond) {
        velocity[i] += ref.velocity[i];
        acceleration[i] = ref.acceleration[i];
      }
    }
    step_towards(velocity, acceleration);
    return true;
  }

  // Twist commands: one cycle towards the reference velocity (the elbow: towards its velocity).
  // Returns true if the reference could not be commanded as it is.
  bool track_velocity(const CartesianArray &v_ref, const CartesianArray &a_ref) {
    update_bounds();
    CartesianArray target = v_ref;
    CartesianArray acceleration = a_ref;
    if (!settings_.use_elbow) {
      target[6] = 0.0;
      acceleration[6] = 0.0;
    }
    const bool clamped = limit_velocity(target, acceleration);
    if (!clamped && feasible(target, direct_ ? 1.0 : kReentryJerkFraction)) {
      accept(target);
      direct_ = true;
      return false;
    }
    direct_ = false;
    step_towards(target, acceleration);
    return true;
  }

  // One cycle of maximal braking towards rest. Returns true at rest.
  bool brake() {
    update_bounds();
    step_towards(CartesianArray{}, CartesianArray{});
    if (at_rest()) {
      hold();
      return true;
    }
    return false;
  }

  // Keeps the current pose. Only meaningful when already at rest.
  void hold() {
    state_.velocity.fill(0.0);
    state_.acceleration.fill(0.0);
    u0_.fill(0.0);
    u1_.fill(0.0);
    direct_ = true;
  }

  bool at_rest() const {
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      if (std::abs(state_.velocity[i]) > kRestVelocity || std::abs(state_.acceleration[i]) > kRestAcceleration) {
        return false;
      }
    }
    return true;
  }

  bool nearly_at_rest() const {
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      if (std::abs(state_.velocity[i]) > 1e-3 || std::abs(state_.acceleration[i]) > 1e-2) {
        return false;
      }
    }
    return true;
  }

  // Commanded state: the pose (integral of the twists for twist commands), the elbow, the twist
  const CartesianState &state() const { return state_; }
  uint64_t errors() const { return errors_; }

  // --- interface used by Pipeline ---
  void configure(const PipelineTypes::Config &config) { set_settings(config.cartesian); }
  void set_feedback(const CartesianFeedback &) {}

  // Pose streams: the quaternion with the sign closest to the previous sample, or to the command
  void align_sample(Values &values, const Values *previous) const {
    if constexpr (kPoseCommands) {
      const Quat reference = previous != nullptr ? Quat{(*previous)[3], (*previous)[4], (*previous)[5], (*previous)[6]}
                                                 : state_.pose.q;
      const Quat q = aligned(Quat{values[3], values[4], values[5], values[6]}, reference);
      for (std::size_t i = 0; i < 4; ++i) {
        values[3 + i] = q[i];
      }
    }
  }

  // A pose stream that (re)starts farther than max_jump (m) or max_jump_angle (rad) from the
  // command is rejected
  bool rejects_stream_start(const Values &values, double &distance) const {
    distance = 0.0;
    if constexpr (kPoseCommands) {
      const Pose ref = pose_of(values);
      distance = (to_eigen(ref.p) - to_eigen(state_.pose.p)).norm();
      const double angle = rotation_between(state_.pose.q, ref.q).norm();
      return distance > settings_.max_jump || angle > settings_.max_jump_angle;
    } else {
      return false;
    }
  }

  // For clamp disabled: a stream reference beyond the limits stops the robot
  bool reference_within_limits(const Values &values, const Values &rates) const {
    const CartesianState ref = reference_of(values, rates, Values{});
    if constexpr (kPoseCommands) {
      if (ref.pose.p[2] < settings_.min_height) {
        return false;
      }
    }
    return norm3(ref.velocity, 0) <= settings_.limits.v_max && norm3(ref.velocity, 3) <= settings_.limits.w_max;
  }

  bool follow_stream(const Values &value, const Values &rate, const Values &rate_dot) {
    const CartesianState ref = reference_of(value, rate, rate_dot);
    if constexpr (kPoseCommands) {
      return track_pose(ref);
    } else {
      return track_velocity(ref.velocity, ref.acceleration);
    }
  }

  // Internal motions start from the command, stay above min_height and within the elbow limits,
  // and are stretched to respect the Cartesian limits. Without elbow control the elbow is kept.
  bool prepare_motion(CartesianMotion &motion) const {
    motion.start = state_.pose;
    motion.start_elbow = state_.elbow;
    if (!settings_.use_elbow || !std::isfinite(motion.goal_elbow)) {
      motion.goal_elbow = state_.elbow;
    }
    if (motion.type == CartesianMotion::Type::kLineBump && motion.axis > 2) {
      return false;
    }
    if (!(motion.min_height() >= settings_.min_height)) {
      return false;
    }
    if (settings_.use_elbow && (motion.goal_elbow < elbow_min() || motion.goal_elbow > elbow_max())) {
      return false;
    }
    motion.duration = std::max({motion.duration, motion.min_duration(settings_.limits), kCycleTime});
    return std::isfinite(motion.duration);
  }

  bool follow_motion(const CartesianState &point) {
    if constexpr (kPoseCommands) {
      return track_pose(point);
    } else {
      // The commanded pose, integral of the twists, is kept on the reference pose
      CartesianArray velocity = point.velocity;
      const CartesianArray correction = correction_towards(point.pose, point.elbow);
      for (std::size_t i = 0; i < kCartesianAxes; ++i) {
        velocity[i] += correction[i];
      }
      return track_velocity(velocity, point.acceleration);
    }
  }

  const CartesianState &command() const { return state_; }
  const CartesianState &position() const { return state_; }

 private:
  static constexpr double kPositionGain = 30.0;  // 1/s, approach of poses and limits
  // A snap at up to this fraction of the jerk limit is undone within the limit by the next samples
  static constexpr double kReentryJerkFraction = 0.3;
  static constexpr double kRestVelocity = 1e-5;
  static constexpr double kRestAcceleration = 1e-4;

  static Pose pose_of(const std::array<double, 7> &values) {
    Pose pose;
    pose.p = {values[0], values[1], values[2]};
    pose.q = normalized(Quat{values[3], values[4], values[5], values[6]});
    return pose;
  }

  // Reference state of a stream sample with its first two derivatives. Streams do not carry the
  // elbow: it is held.
  CartesianState reference_of(const Values &value, const Values &rate, const Values &rate_dot) const {
    CartesianState ref;
    ref.elbow = state_.elbow;
    if constexpr (kPoseCommands) {
      ref.pose = pose_of(value);
      const Eigen::Vector3d w = angular_velocity(ref.pose.q, Quat{rate[3], rate[4], rate[5], rate[6]});
      const Eigen::Vector3d dw = angular_velocity(ref.pose.q, Quat{rate_dot[3], rate_dot[4], rate_dot[5], rate_dot[6]});
      for (std::size_t i = 0; i < 3; ++i) {
        ref.velocity[i] = rate[i];
        ref.acceleration[i] = rate_dot[i];
        ref.velocity[3 + i] = w[i];
        ref.acceleration[3 + i] = dw[i];
      }
    } else {
      ref.pose = state_.pose;
      for (std::size_t i = 0; i < 6; ++i) {
        ref.velocity[i] = value[i];
        ref.acceleration[i] = rate[i];
      }
    }
    return ref;
  }

  double elbow_min() const { return settings_.limits.elbow_min + settings_.elbow_margin; }
  double elbow_max() const { return settings_.limits.elbow_max - settings_.elbow_margin; }

  // Downward speed allowed above min_height and elbow speeds allowed within its limits: braking
  // at half the per-axis acceleration limit, linear in the distance close to the limit
  void update_bounds() {
    auto bound = [](double distance, double v_max, double a_brake) {
      const double d = std::max(0.0, distance);
      return std::min({v_max, std::sqrt(2 * a_brake * d), kPositionGain * d});
    };
    vz_down_ = bound(state_.pose.p[2] - settings_.min_height, v_axis_[2], 0.5 * a_axis_[2]);
    if (settings_.use_elbow) {
      elbow_up_ = bound(elbow_max() - state_.elbow, v_axis_[6], 0.5 * a_axis_[6]);
      elbow_down_ = bound(state_.elbow - elbow_min(), v_axis_[6], 0.5 * a_axis_[6]);
    } else {
      elbow_up_ = elbow_down_ = 0.0;
    }
  }

  // Scales the translational and rotational velocities to their norm limits and applies the
  // height and elbow bounds. Removes the acceleration feedforward of what was limited. Returns
  // true if anything was limited.
  bool limit_velocity(CartesianArray &velocity, CartesianArray &acceleration) const {
    bool limited = false;
    const double limits[2] = {settings_.limits.v_max, settings_.limits.w_max};
    for (std::size_t group = 0; group < 2; ++group) {
      const std::size_t first = 3 * group;
      const double norm = norm3(velocity, first);
      if (norm > limits[group]) {
        for (std::size_t i = first; i < first + 3; ++i) {
          velocity[i] *= limits[group] / norm;
          acceleration[i] = 0.0;
        }
        limited = true;
      }
    }
    if (velocity[2] < -vz_down_) {
      velocity[2] = -vz_down_;
      acceleration[2] = 0.0;
      limited = true;
    }
    const double elbow = std::clamp(velocity[6], -elbow_down_, elbow_up_);
    if (elbow != velocity[6]) {
      velocity[6] = elbow;
      acceleration[6] = 0.0;
      limited = true;
    }
    return limited;
  }

  // Velocity of a candidate pose as the robot computes it: finite differences of the position and
  // rotation vector of the relative orientation, in the base frame
  CartesianArray velocity_to(const Pose &target, double elbow) const {
    CartesianArray u{};
    const Eigen::Vector3d r = rotation_between(state_.pose.q, target.q);
    for (std::size_t i = 0; i < 3; ++i) {
      u[i] = (target.p[i] - state_.pose.p[i]) / dt_;
      u[3 + i] = r[i] / dt_;
    }
    u[6] = (elbow - state_.elbow) / dt_;
    return u;
  }

  // Velocity bringing the command to the target pose: linear in the error close to it,
  // braking-limited far from it
  CartesianArray correction_towards(const Pose &target, double elbow) const {
    auto correction = [](double error, double a_brake) {
      return std::min(kPositionGain * error, std::sqrt(2 * a_brake * error));
    };
    CartesianArray c{};
    const Eigen::Vector3d ep = to_eigen(target.p) - to_eigen(state_.pose.p);
    const Eigen::Vector3d er = rotation_between(state_.pose.q, target.q);
    const double np = ep.norm(), nr = er.norm();
    for (std::size_t i = 0; i < 3; ++i) {
      c[i] = np > 0.0 ? ep[i] / np * correction(np, 0.5 * a_axis_[i]) : 0.0;
      c[3 + i] = nr > 0.0 ? er[i] / nr * correction(nr, 0.5 * a_axis_[3 + i]) : 0.0;
    }
    if (settings_.use_elbow) {
      const double ee = elbow - state_.elbow;
      c[6] = std::copysign(correction(std::abs(ee), 0.5 * a_axis_[6]), ee);
    }
    return c;
  }

  // Finite differences of the candidate velocity against the last commands, as checked by the
  // robot: norms of velocity, acceleration and jerk
  bool feasible(const CartesianArray &u, double jerk_fraction) const {
    const CartesianLimits &l = settings_.limits;
    if (norm3(u, 0) > l.v_max || norm3(u, 3) > l.w_max || std::abs(u[6]) > l.elbow_v_max) {
      return false;
    }
    if (u[2] < -vz_down_ || u[6] > elbow_up_ || u[6] < -elbow_down_) {
      return false;
    }
    CartesianArray a{}, j{};
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      a[i] = (u[i] - u0_[i]) / dt_;
      j[i] = (a[i] - (u0_[i] - u1_[i]) / dt_) / dt_;
    }
    if (norm3(a, 0) > l.a_max || norm3(a, 3) > l.dw_max || std::abs(a[6]) > l.elbow_a_max) {
      return false;
    }
    return norm3(j, 0) <= jerk_fraction * l.j_max && norm3(j, 3) <= jerk_fraction * l.ddw_max &&
           std::abs(j[6]) <= jerk_fraction * l.elbow_j_max;
  }

  // A velocity commanded as it is: history, acceleration and the integrated pose
  void accept(const CartesianArray &u) {
    u1_ = u0_;
    u0_ = u;
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      state_.acceleration[i] = (u0_[i] - u1_[i]) / dt_;
    }
    state_.velocity = u;
    integrate(u);
  }

  // As the robot integrates the commanded twist: one command per cycle
  void integrate(const CartesianArray &u) {
    for (std::size_t i = 0; i < 3; ++i) {
      state_.pose.p[i] += u[i] * dt_;
    }
    state_.pose.q = rotated(state_.pose.q, Eigen::Vector3d(u[3], u[4], u[5]) * dt_);
    state_.elbow += u[6] * dt_;
  }

  // One Ruckig velocity-interface step towards the target velocity, within the limits
  void step_towards(CartesianArray velocity, CartesianArray acceleration) {
    if (!settings_.use_elbow) {
      velocity[6] = 0.0;
      acceleration[6] = 0.0;
    }
    limit_velocity(velocity, acceleration);
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      // Ruckig works on the velocities only: the pose is integrated here
      input_.current_position[i] = 0.0;
      input_.current_velocity[i] = state_.velocity[i];
      input_.current_acceleration[i] = state_.acceleration[i];
      input_.target_velocity[i] = std::clamp(velocity[i], -v_axis_[i], v_axis_[i]);
      input_.target_acceleration[i] = std::clamp(acceleration[i], -a_axis_[i], a_axis_[i]);
      input_.max_velocity[i] = v_axis_[i];
      input_.max_acceleration[i] = a_axis_[i];
      input_.max_jerk[i] = j_axis_[i];
    }
    // A target acceleration at the limit can leave Ruckig without a profile: then aim at the target
    // velocity alone, always reachable from a state within the limits
    if (otg_.update(input_, output_) < 0) {
      input_.target_acceleration.fill(0.0);
      if (otg_.update(input_, output_) < 0) {
        // Should not happen with valid limits: keep the velocity rather than command an undefined state
        ++errors_;
        accept(state_.velocity);
        return;
      }
    }
    CartesianArray u;
    for (std::size_t i = 0; i < kCartesianAxes; ++i) {
      u[i] = output_.new_velocity[i];
      state_.acceleration[i] = output_.new_acceleration[i];
    }
    u1_ = u0_;
    u0_ = u;
    state_.velocity = u;
    integrate(u);
  }

  ruckig::Ruckig<kCartesianAxes> otg_;
  double dt_;
  ruckig::InputParameter<kCartesianAxes> input_;
  ruckig::OutputParameter<kCartesianAxes> output_;
  CartesianSettings settings_{};
  CartesianArray v_axis_{}, a_axis_{}, j_axis_{};
  CartesianState state_{};
  // Last two commanded velocities, newest first
  CartesianArray u0_{}, u1_{};
  double vz_down_{0.0}, elbow_up_{0.0}, elbow_down_{0.0};
  // True while the reference is commanded as it is
  bool direct_{true};
  uint64_t errors_{0};
};

using CartesianPoseTracker = CartesianTracker<true>;
using CartesianVelocityTracker = CartesianTracker<false>;

}  // namespace panda_bridge
