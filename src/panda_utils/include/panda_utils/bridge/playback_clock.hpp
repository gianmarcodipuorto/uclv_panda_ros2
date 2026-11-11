#pragma once

#include <cmath>
#include <cstdint>

namespace panda_bridge {

// Time base of the command playback. The robot consumes one command every millisecond of its
// own clock, while the PC receives its packets with tens of microseconds of jitter: sampling the
// reference at the time of arrival would turn that jitter into position noise, and its finite
// differences into large jerk. This clock advances with the robot time (exactly 1 ms per cycle)
// and is aligned to the PC wall clock, the time base of the header stamps, by a slow first-order
// loop that follows the drift between the two clocks without passing the jitter.
class PlaybackClock {
 public:
  void reset(int64_t wall_ns, int64_t robot_ns) {
    base_ns_ = wall_ns - robot_ns;
    residual_ns_ = 0.0;
  }

  // Wall-clock time aligned with the given robot time.
  int64_t update(int64_t wall_ns, int64_t robot_ns) {
    // The offset is an integer base plus a small residual: wall-clock nanoseconds (~1e18) do
    // not fit a double with the resolution needed by the sub-nanosecond corrections
    const double error = static_cast<double>(wall_ns - robot_ns - base_ns_) - residual_ns_;
    if (std::abs(error) > kResyncNs) {
      // Clock step (e.g. NTP): realign at once
      reset(wall_ns, robot_ns);
    } else {
      residual_ns_ += kGain * error;
      const auto whole = static_cast<int64_t>(residual_ns_);
      base_ns_ += whole;
      residual_ns_ -= static_cast<double>(whole);
    }
    return robot_ns + base_ns_ + static_cast<int64_t>(std::llround(residual_ns_));
  }

 private:
  // Time constant of 1000 control cycles, about one second
  static constexpr double kGain = 1e-3;
  static constexpr double kResyncNs = 20e6;
  int64_t base_ns_{0};
  double residual_ns_{0.0};
};

}  // namespace panda_bridge
