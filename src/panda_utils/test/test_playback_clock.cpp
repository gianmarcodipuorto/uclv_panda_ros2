#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "kinematics_checks.hpp"
#include "panda_utils/bridge/pipeline.hpp"
#include "panda_utils/bridge/playback_clock.hpp"

using panda_bridge::JointArray;
using panda_bridge::JointMotion;
using panda_bridge::JointPositionPipeline;
using panda_bridge::kCycleNs;
using panda_bridge::PlaybackClock;

namespace {

constexpr int64_t kWallOrigin = 1'700'000'000'000'000'000LL;

}  // namespace

TEST(PlaybackClock, RemovesArrivalJitter) {
  PlaybackClock clock;
  std::mt19937 rng(3);
  std::normal_distribution<double> jitter(0.0, 50e3);  // 50 us
  clock.reset(kWallOrigin, 0);
  int64_t previous = kWallOrigin;
  for (int k = 1; k <= 5000; ++k) {
    const int64_t robot = k * kCycleNs;
    const int64_t wall = kWallOrigin + robot + static_cast<int64_t>(jitter(rng));
    const int64_t aligned = clock.update(wall, robot);
    // Steps of one millisecond, within a fraction of a microsecond
    EXPECT_NEAR(static_cast<double>(aligned - previous), 1e6, 500.0) << "cycle " << k;
    previous = aligned;
  }
}

TEST(PlaybackClock, FollowsClockDrift) {
  PlaybackClock clock;
  clock.reset(kWallOrigin, 0);
  int64_t aligned = 0;
  int64_t wall = 0;
  // The PC clock runs 50 ppm faster than the robot clock
  for (int k = 1; k <= 20000; ++k) {
    const int64_t robot = k * kCycleNs;
    wall = kWallOrigin + robot + robot / 20000;
    aligned = clock.update(wall, robot);
  }
  EXPECT_NEAR(static_cast<double>(aligned - wall), 0.0, 100e3);  // lag below 0.1 ms
}

TEST(PlaybackClock, ResynchronizesAfterClockStep) {
  PlaybackClock clock;
  clock.reset(kWallOrigin, 0);
  clock.update(kWallOrigin + kCycleNs, kCycleNs);
  const int64_t wall = kWallOrigin + 2 * kCycleNs + 500'000'000;  // the PC clock jumps by 0.5 s
  EXPECT_EQ(clock.update(wall, 2 * kCycleNs), wall);
}

// End to end: the robot loop runs with arrival jitter, the pipeline plays back on the aligned
// clock, and the commands stay as smooth as the stream.
TEST(PlaybackClock, PipelineCommandsStaySmoothWithJitteryCallbacks) {
  JointPositionPipeline pipeline;
  JointPositionPipeline::Config config;
  config.limits = panda_bridge::fer_joint_limits(0.9, 0.8, 0.5);
  pipeline.configure(config);
  const JointArray start = panda_bridge::test::ready_pose();
  JointMotion trajectory;
  trajectory.start = start;
  trajectory.goal = start;
  trajectory.goal[0] += 0.4;
  trajectory.duration = 3.0;

  PlaybackClock clock;
  clock.reset(kWallOrigin, 0);
  pipeline.reset(start, JointArray{}, JointArray{}, kWallOrigin);
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> jitter(0.0, 200e3);  // callbacks up to 0.2 ms late
  std::vector<JointArray> commands{start, start, start};
  for (int k = 1; k <= 3500; ++k) {
    const int64_t robot = k * kCycleNs;
    // The client samples the trajectory on the wall clock, 1 kHz
    JointPositionPipeline::Sample sample;
    sample.stamp_ns = kWallOrigin + robot;
    JointArray v, a;
    trajectory.evaluate(k * 1e-3, sample.values, v, a);
    pipeline.push_sample(sample);
    const int64_t wall = kWallOrigin + robot + static_cast<int64_t>(jitter(rng));
    pipeline.step(clock.update(wall, robot));
    commands.push_back(pipeline.command());
  }
  EXPECT_EQ(pipeline.counters().limited_cycles, 0u);
  panda_bridge::test::expect_within_limits(commands, config.limits);
  // Jerk well below the limits: the jitter does not reach the commands
  for (std::size_t k = 3; k < commands.size(); ++k) {
    const double j = (commands[k][0] - 3 * commands[k - 1][0] + 3 * commands[k - 2][0] - commands[k - 3][0]) / 1e-9;
    ASSERT_LT(std::abs(j), 100.0) << "cycle " << k;
  }
}
