#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "panda_utils/bridge/cartesian_types.hpp"
#include "panda_utils/bridge/franka_limits.hpp"
#include "panda_utils/bridge/pipeline_types.hpp"

namespace panda_bridge {

// Cartesian motions generated inside the real-time loop (internal mode). The profiles start and
// end at rest, so they can be chained through the hold state. The orientation is kept, except
// by the quintic.
struct CartesianMotion {
  enum class Type : uint8_t {
    kQuintic,   // rest-to-rest from start to goal: straight line, slerp, elbow
    kLineBump,  // out along a base axis by amplitude and back: start + A/2 (1 - cos(2 pi t / T))
    kCircle,    // one turn of radius amplitude in the base xy plane, through start, rest to rest
  };

  Type type{Type::kQuintic};
  Pose start{};
  double start_elbow{0.0};
  Pose goal{};
  double goal_elbow{0.0};
  double duration{1.0};
  std::size_t axis{0};    // kLineBump: 0 x, 1 y, 2 z
  double amplitude{0.0};  // m

  bool is_free() const { return false; }

  void evaluate(double t, CartesianState &point) const {
    point = CartesianState{};
    point.pose = start;
    point.elbow = start_elbow;
    const double T = duration;
    if (type == Type::kQuintic) {
      double s, ds, dds;
      quintic(t, s, ds, dds);
      for (std::size_t i = 0; i < 3; ++i) {
        const double d = goal.p[i] - start.p[i];
        point.pose.p[i] = start.p[i] + d * s;
        point.velocity[i] = d * ds;
        point.acceleration[i] = d * dds;
      }
      // Slerp: rotation about a fixed axis of the base frame
      const Eigen::Vector3d r = rotation_between(start.q, goal.q);
      point.pose.q = rotated(start.q, r * s);
      for (std::size_t i = 0; i < 3; ++i) {
        point.velocity[3 + i] = r[i] * ds;
        point.acceleration[3 + i] = r[i] * dds;
      }
      const double de = goal_elbow - start_elbow;
      point.elbow = start_elbow + de * s;
      point.velocity[6] = de * ds;
      point.acceleration[6] = de * dds;
    } else if (type == Type::kLineBump && t > 0.0 && t < T) {
      const double w = 2 * M_PI / T;
      point.pose.p[axis] += 0.5 * amplitude * (1 - std::cos(w * t));
      point.velocity[axis] = 0.5 * amplitude * w * std::sin(w * t);
      point.acceleration[axis] = 0.5 * amplitude * w * w * std::cos(w * t);
    } else if (type == Type::kCircle) {
      double s, ds, dds;
      quintic(t, s, ds, dds);
      const double theta = 2 * M_PI * s, dtheta = 2 * M_PI * ds, ddtheta = 2 * M_PI * dds;
      const double c = std::cos(theta), sn = std::sin(theta);
      // Center on the -x side, so that the circle starts at the start position
      point.pose.p[0] = start.p[0] - amplitude + amplitude * c;
      point.pose.p[1] = start.p[1] + amplitude * sn;
      point.velocity[0] = -amplitude * dtheta * sn;
      point.velocity[1] = amplitude * dtheta * c;
      point.acceleration[0] = -amplitude * (ddtheta * sn + dtheta * dtheta * c);
      point.acceleration[1] = amplitude * (ddtheta * c - dtheta * dtheta * sn);
    }
  }

  // Shortest duration keeping the peaks of the profile within the limits.
  double min_duration(const CartesianLimits &limits) const {
    // Peaks of |v|, |a|, |j| per unit of displacement are kv/T, ka/T^2, kj/T^3
    auto shortest = [](double d, double kv, double ka, double kj, double v_max, double a_max, double j_max) {
      if (!(d > 0.0)) {
        return 0.0;
      }
      return std::max({kv * d / v_max, std::sqrt(ka * d / a_max), std::cbrt(kj * d / j_max)});
    };
    if (type == Type::kQuintic) {
      const double distance = (to_eigen(goal.p) - to_eigen(start.p)).norm();
      const double angle = rotation_between(start.q, goal.q).norm();
      const double elbow = std::abs(goal_elbow - start_elbow);
      return std::max(
          {shortest(distance, 1.875, 5.7735, 60.0, limits.v_max, limits.a_max, limits.j_max),
           shortest(angle, 1.875, 5.7735, 60.0, limits.w_max, limits.dw_max, limits.ddw_max),
           shortest(elbow, 1.875, 5.7735, 60.0, limits.elbow_v_max, limits.elbow_a_max, limits.elbow_j_max)});
    }
    if (type == Type::kLineBump) {
      return shortest(std::abs(amplitude), M_PI, 2 * M_PI * M_PI, 4 * M_PI * M_PI * M_PI, limits.v_max,
                      limits.a_max, limits.j_max);
    }
    // Circle: bounds of r dtheta, r (ddtheta + dtheta^2), r (dddtheta + 3 dtheta ddtheta + dtheta^3)
    return shortest(std::abs(amplitude), 11.781, 175.07, 3294.2, limits.v_max, limits.a_max, limits.j_max);
  }

  // Lowest height of the end effector along the motion.
  double min_height() const {
    if (type == Type::kQuintic) {
      return std::min(start.p[2], goal.p[2]);
    }
    if (type == Type::kLineBump && axis == 2) {
      return start.p[2] + std::min(0.0, amplitude);
    }
    return start.p[2];
  }

 private:
  void quintic(double t, double &s, double &ds, double &dds) const {
    const double T = duration;
    const double tau = std::clamp(t / T, 0.0, 1.0);
    const double tau2 = tau * tau, tau3 = tau2 * tau;
    s = tau3 * (10 - 15 * tau + 6 * tau2);
    ds = 30 * tau2 * (1 - 2 * tau + tau2) / T;
    dds = 60 * tau * (1 - 3 * tau + 2 * tau2) / (T * T);
  }
};

using CartesianMotionRequest = MotionRequest<CartesianMotion>;

}  // namespace panda_bridge
