#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "panda_utils/bridge/types.hpp"

namespace panda_bridge {

// Time-indexed buffer of streamed setpoints. Samples are played back at a fixed delay behind the
// wall clock: the reference is interpolated between the two samples that bracket the playback
// time, so late, duplicated or missing messages of the transport do not reach the robot.
// Fixed capacity, no allocations: usable from the real-time loop.
template <std::size_t N, std::size_t Capacity = 64>
class JitterBuffer {
 public:
  using Values = std::array<double, N>;

  struct Sample {
    int64_t stamp_ns{0};
    Values values{};
  };

  enum class Status : uint8_t {
    kEmpty,         // no samples
    kWaiting,       // playback time older than the oldest sample (stream just started)
    kInterpolated,  // playback time between two samples
    kStarved,       // playback time newer than the newest sample
  };

  struct Reference {
    Status status{Status::kEmpty};
    Values value{};
    Values rate{};      // first time derivative
    Values rate_dot{};  // second time derivative
    int64_t age_ns{0};  // playback time minus newest stamp, > 0 when starved
  };

  void clear() {
    head_ = 0;
    count_ = 0;
  }

  std::size_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  const Sample &oldest() const { return at(0); }
  const Sample &newest() const { return at(count_ - 1); }

  // Samples not newer than the newest one are rejected (duplicates, reordering).
  // When full, the oldest sample is overwritten.
  bool push(const Sample &sample) {
    if (count_ > 0 && sample.stamp_ns <= newest().stamp_ns) {
      return false;
    }
    if (count_ == Capacity) {
      pop_oldest();
    }
    ring_[(head_ + count_) % Capacity] = sample;
    ++count_;
    return true;
  }

  // Local quadratic least-squares fit around the playback time, for positions: value and first
  // two derivatives stay smooth even when samples are missing or irregularly spaced (an
  // interpolating spline would have acceleration jumps at every sample, i.e. jerk spikes).
  // Uses the samples within half_window_ns of t_ns, widened for sparse streams. When starved,
  // the fit at the newest sample is extrapolated to t_ns: with its second derivative (positions:
  // constant acceleration), or without it (velocities: constant acceleration too, not jerk).
  // Queries must be monotonic: samples no longer needed are discarded.
  Reference sample_fit(int64_t t_ns, int64_t half_window_ns, bool extrapolate_second_derivative = true) {
    Reference ref;
    const bool bracketed = locate(t_ns, half_window_ns, ref);
    if (ref.status == Status::kStarved) {
      const int64_t t_newest = newest().stamp_ns;
      Reference at_newest;
      if (fit(t_newest, widened(half_window_ns, count_ - 1), at_newest)) {
        const double age = ns_to_s(ref.age_ns);
        for (std::size_t j = 0; j < N; ++j) {
          const double curvature = extrapolate_second_derivative ? at_newest.rate_dot[j] : 0.0;
          ref.value[j] = at_newest.value[j] + at_newest.rate[j] * age + 0.5 * curvature * age * age;
          ref.rate[j] = at_newest.rate[j] + curvature * age;
          ref.rate_dot[j] = curvature;
        }
      } else {
        for (std::size_t j = 0; j < N; ++j) {
          ref.value[j] += ref.rate[j] * ns_to_s(ref.age_ns);
        }
      }
      return ref;
    }
    if (!bracketed) {
      return ref;
    }
    const std::size_t i = lower_index(t_ns);
    if (!fit(t_ns, widened(half_window_ns, i + 1), ref)) {
      interpolate_linear(t_ns, i, ref);
    }
    return ref;
  }

  // Piecewise-linear interpolation, for velocities and torques.
  Reference sample_linear(int64_t t_ns) {
    Reference ref;
    if (!locate(t_ns, 0, ref)) {
      return ref;
    }
    return interpolate_linear(t_ns, lower_index(t_ns), ref);
  }

 private:
  const Sample &at(std::size_t i) const { return ring_[(head_ + i) % Capacity]; }

  void pop_oldest() {
    head_ = (head_ + 1) % Capacity;
    --count_;
  }

  // Fit window: at least two and a half sample intervals on each side, for sparse streams.
  int64_t widened(int64_t half_window_ns, std::size_t i) const {
    const int64_t interval = i > 0 ? at(i).stamp_ns - at(i - 1).stamp_ns : 0;
    return std::max(half_window_ns, interval * 5 / 2);
  }

  // Least-squares fit of p(tau) = c0 + c1 tau + c2 tau^2 to the samples within half_ns of t_ns.
  // Returns false with fewer than three samples.
  bool fit(int64_t t_ns, int64_t half_ns, Reference &ref) const {
    // Normal equations, tau in ms for conditioning
    double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    Values b0{}, b1{}, b2{};
    for (std::size_t k = 0; k < count_; ++k) {
      const int64_t d = at(k).stamp_ns - t_ns;
      if (d < -half_ns || d > half_ns) {
        continue;
      }
      const double tau = static_cast<double>(d) * 1e-6;
      const double tau2 = tau * tau;
      s0 += 1;
      s1 += tau;
      s2 += tau2;
      s3 += tau2 * tau;
      s4 += tau2 * tau2;
      for (std::size_t j = 0; j < N; ++j) {
        const double v = at(k).values[j];
        b0[j] += v;
        b1[j] += v * tau;
        b2[j] += v * tau2;
      }
    }
    // Cofactors of the symmetric 3x3 matrix
    const double m00 = s2 * s4 - s3 * s3, m01 = s2 * s3 - s1 * s4, m02 = s1 * s3 - s2 * s2;
    const double m11 = s0 * s4 - s2 * s2, m12 = s1 * s2 - s0 * s3, m22 = s0 * s2 - s1 * s1;
    const double det = s0 * m00 + s1 * m01 + s2 * m02;
    if (s0 < 3 || std::abs(det) < 1e-12) {
      return false;
    }
    for (std::size_t j = 0; j < N; ++j) {
      ref.value[j] = (m00 * b0[j] + m01 * b1[j] + m02 * b2[j]) / det;
      ref.rate[j] = (m01 * b0[j] + m11 * b1[j] + m12 * b2[j]) / det * 1e3;
      ref.rate_dot[j] = 2 * (m02 * b0[j] + m12 * b1[j] + m22 * b2[j]) / det * 1e6;
    }
    return true;
  }

  Reference &interpolate_linear(int64_t t_ns, std::size_t i, Reference &ref) const {
    const Sample &s0 = at(i);
    const Sample &s1 = at(i + 1);
    const double h = ns_to_s(s1.stamp_ns - s0.stamp_ns);
    const double s = ns_to_s(t_ns - s0.stamp_ns) / h;
    for (std::size_t k = 0; k < N; ++k) {
      ref.value[k] = s0.values[k] + s * (s1.values[k] - s0.values[k]);
      ref.rate[k] = (s1.values[k] - s0.values[k]) / h;
      ref.rate_dot[k] = 0.0;
    }
    return ref;
  }

  // Handles the cases without a bracketing pair. Returns true when interpolation is possible.
  // Discards the samples older than the fit window around min(t_ns, newest), except two.
  bool locate(int64_t t_ns, int64_t half_window_ns, Reference &ref) {
    if (count_ == 0) {
      ref.status = Status::kEmpty;
      return false;
    }
    const int64_t keep_from = std::min(t_ns, newest().stamp_ns) - 2 * widened(half_window_ns, count_ - 1);
    while (count_ >= 3 && at(2).stamp_ns <= keep_from) {
      pop_oldest();
    }
    if (t_ns < oldest().stamp_ns) {
      ref.status = Status::kWaiting;
      ref.value = oldest().values;
      return false;
    }
    if (t_ns >= newest().stamp_ns) {
      ref.status = Status::kStarved;
      ref.value = newest().values;
      ref.age_ns = t_ns - newest().stamp_ns;
      for (std::size_t k = 0; k < N; ++k) {
        ref.rate[k] = count_ >= 2 ? secant(count_ - 2, count_ - 1, k) : 0.0;
      }
      return false;
    }
    ref.status = Status::kInterpolated;
    return true;
  }

  // Last sample with stamp <= t_ns; requires oldest <= t_ns < newest.
  std::size_t lower_index(int64_t t_ns) const {
    std::size_t i = 0;
    while (i + 2 < count_ && at(i + 1).stamp_ns <= t_ns) {
      ++i;
    }
    return i;
  }

  double secant(std::size_t a, std::size_t b, std::size_t k) const {
    return (at(b).values[k] - at(a).values[k]) / ns_to_s(at(b).stamp_ns - at(a).stamp_ns);
  }

  std::array<Sample, Capacity> ring_{};
  std::size_t head_{0};
  std::size_t count_{0};
};

}  // namespace panda_bridge
