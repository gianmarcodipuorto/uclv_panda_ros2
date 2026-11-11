#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "panda_utils/bridge/franka_limits.hpp"
#include "panda_utils/bridge/pipeline_types.hpp"

namespace panda_bridge {

// Reference of a joint motion at one instant.
struct JointPoint {
  JointArray p{};
  JointArray v{};
  JointArray a{};
};

// Joint-space motions generated inside the real-time loop (internal mode). The profiles start and
// end at rest, so they can be chained through the hold state.
struct JointMotion {
  enum class Type : uint8_t {
    kQuintic,     // rest-to-rest quintic from start to goal
    kCosineBump,  // one joint goes out by amplitude and back: start + A/2 (1 - cos(2 pi t / T))
    kFree,        // no reference, until stopped: gravity compensation (torque bridge only)
  };

  Type type{Type::kQuintic};
  JointArray start{};
  JointArray goal{};
  double duration{1.0};
  std::size_t joint{0};
  double amplitude{0.0};

  void evaluate(double t, JointArray &p, JointArray &v, JointArray &a) const {
    p = start;
    v.fill(0.0);
    a.fill(0.0);
    const double T = duration;
    if (type == Type::kQuintic) {
      const double tau = std::clamp(t / T, 0.0, 1.0);
      const double tau2 = tau * tau, tau3 = tau2 * tau;
      const double s = tau3 * (10 - 15 * tau + 6 * tau2);
      const double ds = 30 * tau2 * (1 - 2 * tau + tau2) / T;
      const double dds = 60 * tau * (1 - 3 * tau + 2 * tau2) / (T * T);
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double d = goal[i] - start[i];
        p[i] = start[i] + d * s;
        v[i] = d * ds;
        a[i] = d * dds;
      }
    } else if (type == Type::kCosineBump && t > 0.0 && t < T) {
      const double w = 2 * M_PI / T;
      p[joint] = start[joint] + 0.5 * amplitude * (1 - std::cos(w * t));
      v[joint] = 0.5 * amplitude * w * std::sin(w * t);
      a[joint] = 0.5 * amplitude * w * w * std::cos(w * t);
    }
  }

  void evaluate(double t, JointPoint &point) const { evaluate(t, point.p, point.v, point.a); }

  bool is_free() const { return type == Type::kFree; }

  const JointArray &final_position() const { return type == Type::kQuintic ? goal : start; }

  // Shortest duration keeping the peaks of the profile within the bounds.
  double min_duration(const JointArray &v_max, const JointArray &a_max, const JointArray &j_max) const {
    double t_min = 0.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      double d = 0.0;
      // Peak |v|, |a| and |j| per unit displacement: quintic 1.875/T, 5.7735/T^2, 60/T^3;
      // cosine bump pi/T, 2 pi^2/T^2, 4 pi^3/T^3
      double kv = 0.0, ka = 0.0, kj = 0.0;
      if (type == Type::kQuintic) {
        d = std::abs(goal[i] - start[i]);
        kv = 1.875;
        ka = 5.7735;
        kj = 60.0;
      } else if (type == Type::kCosineBump && i == joint) {
        d = std::abs(amplitude);
        kv = M_PI;
        ka = 2 * M_PI * M_PI;
        kj = 4 * M_PI * M_PI * M_PI;
      }
      if (d > 0.0) {
        t_min = std::max({t_min, kv * d / v_max[i], std::sqrt(ka * d / a_max[i]), std::cbrt(kj * d / j_max[i])});
      }
    }
    return t_min;
  }
};

using JointMotionRequest = MotionRequest<JointMotion>;

// Prepares a motion requested to a joint tracker: it starts from the current command, must stay
// within the joint limits and is stretched to respect the kinematic limits. False rejects it.
inline bool prepare_joint_motion(JointMotion &motion, const JointArray &start, const JointLimits &limits,
                                 double margin, bool supports_free) {
  motion.start = start;
  if (motion.is_free()) {
    motion.duration = std::numeric_limits<double>::infinity();
    return supports_free;
  }
  if (motion.joint >= kNumJoints) {
    return false;
  }
  JointArray farthest = motion.goal;
  if (motion.type == JointMotion::Type::kCosineBump) {
    farthest = motion.start;
    farthest[motion.joint] += motion.amplitude;
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (farthest[i] < limits.q_min[i] + margin || farthest[i] > limits.q_max[i] - margin) {
      return false;
    }
  }
  motion.duration =
      std::max({motion.duration, motion.min_duration(limits.dq_max, limits.ddq_max, limits.dddq_max), kCycleTime});
  return true;
}

}  // namespace panda_bridge
