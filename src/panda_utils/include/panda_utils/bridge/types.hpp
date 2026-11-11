#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>

namespace panda_bridge {

constexpr std::size_t kNumJoints = 7;
// libfranka control period
constexpr double kCycleTime = 1e-3;
constexpr int64_t kCycleNs = 1'000'000;

using JointArray = std::array<double, kNumJoints>;

// Joint quantities of the robot state used by the pipelines.
struct JointFeedback {
  // Measured
  JointArray q{};
  JointArray dq{};
  // Last command of the motion generator
  JointArray q_d{};
  JointArray dq_d{};
  JointArray ddq_d{};
  // Last commanded torque, without gravity
  JointArray tau_d{};
  // Coriolis torque from the dynamic model, zero without a model
  JointArray coriolis{};
};

template <std::size_t N>
inline bool all_finite(const std::array<double, N> &values) {
  for (double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

inline double ns_to_s(int64_t ns) { return static_cast<double>(ns) * 1e-9; }

// Wall clock in ns, same time base as ROS header stamps (RCL_ROS_TIME without sim time).
// clock_gettime is served by the vDSO, so it is safe to call from the real-time loop.
inline int64_t wall_clock_ns() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

inline int64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

}  // namespace panda_bridge
