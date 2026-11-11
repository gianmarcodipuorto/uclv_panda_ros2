#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <random>
#include <vector>

#include "kinematics_checks.hpp"
#include "panda_utils/bridge/pipeline.hpp"

using panda_bridge::JointArray;
using panda_bridge::JointMotion;
using panda_bridge::JointMotionRequest;
using panda_bridge::JointVelocityPipeline;
using panda_bridge::kCycleNs;
using panda_bridge::kNumJoints;
using panda_bridge::test::expect_velocities_within_limits;
using panda_bridge::test::ready_pose;

namespace {

using State = JointVelocityPipeline::State;
using Outcome = JointVelocityPipeline::Outcome;
using FinishReason = JointVelocityPipeline::FinishReason;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kStart = 1'000'000'000'000LL;  // arbitrary wall clock origin

JointVelocityPipeline::Config default_config(JointVelocityPipeline::Mode mode) {
  JointVelocityPipeline::Config config;
  config.mode = mode;
  config.limits = panda_bridge::fer_joint_limits(0.9, 0.8, 0.5);
  return config;
}

// Simulated client: the velocity of a quintic, sampled every millisecond and delivered with
// random latency, losses and duplicates. The robot clock runs at 1 kHz.
struct StreamSimulation {
  JointVelocityPipeline pipeline;
  std::vector<JointArray> commands;
  JointMotion trajectory;
  std::mt19937 rng{7};

  struct InFlight {
    int64_t delivery_ns;
    JointVelocityPipeline::Sample sample;
  };
  std::deque<InFlight> in_flight;

  explicit StreamSimulation(const JointVelocityPipeline::Config &config) {
    pipeline.configure(config);
    const JointArray start = ready_pose();
    pipeline.reset(start, JointArray{}, JointArray{}, kStart);
    commands = {JointArray{}, JointArray{}};
    trajectory.start = start;
    trajectory.goal = start;
    trajectory.goal[0] += 0.6;
    trajectory.goal[1] -= 0.3;
    trajectory.goal[5] += 0.4;
    trajectory.duration = 3.0;
  }

  void run(int duration_ms, int publish_until_ms, double max_latency_ms, double loss, double duplicates) {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    for (int k = 1; k <= duration_ms; ++k) {
      const int64_t now = kStart + k * kCycleNs;
      if (k <= publish_until_ms && uniform(rng) >= loss) {
        JointVelocityPipeline::Sample sample;
        sample.stamp_ns = now;
        JointArray p, a;
        trajectory.evaluate((k * 1e-3) - 0.1, p, sample.values, a);  // client starts after 100 ms
        const auto latency = static_cast<int64_t>(uniform(rng) * max_latency_ms * kMs);
        in_flight.push_back({now + latency, sample});
        if (uniform(rng) < duplicates) {
          in_flight.push_back({now + latency + kMs, sample});
        }
      }
      std::stable_sort(in_flight.begin(), in_flight.end(),
                       [](const InFlight &a, const InFlight &b) { return a.delivery_ns < b.delivery_ns; });
      while (!in_flight.empty() && in_flight.front().delivery_ns <= now) {
        pipeline.push_sample(in_flight.front().sample);
        in_flight.pop_front();
      }
      pipeline.step(now);
      commands.push_back(pipeline.command());
    }
  }

  // Largest deviation from the reference velocity delayed by the playout delay.
  double max_tracking_error(int from_ms, int to_ms, int delay_ms) const {
    double error = 0.0;
    for (int k = from_ms; k < to_ms; ++k) {
      JointArray p, v, a;
      trajectory.evaluate((k - delay_ms) * 1e-3 - 0.1, p, v, a);
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        error = std::max(error, std::abs(commands[k + 1][i] - v[i]));
      }
    }
    return error;
  }
};

int run_until(JointVelocityPipeline &pipeline, std::vector<JointArray> &commands, int64_t &now,
              const std::function<bool()> &done, int max_cycles = 20000) {
  int cycles = 0;
  while (!done() && cycles < max_cycles) {
    now += kCycleNs;
    pipeline.step(now);
    commands.push_back(pipeline.command());
    ++cycles;
  }
  return cycles;
}

JointMotionRequest quintic_request(uint64_t id, const JointArray &goal, double duration) {
  JointMotionRequest request;
  request.id = id;
  request.motion.type = JointMotion::Type::kQuintic;
  request.motion.goal = goal;
  request.motion.duration = duration;
  request.name.set("test");
  return request;
}

}  // namespace

TEST(JointVelocityPipeline, FollowsJitteryStreamWithinLimits) {
  StreamSimulation sim(default_config(JointVelocityPipeline::Mode::kExternal));
  // Up to 4 ms of latency, 5% losses and 5% duplicates
  sim.run(4000, 3200, 4.0, 0.05, 0.05);
  expect_velocities_within_limits(sim.commands, sim.pipeline.config().limits);
  EXPECT_LT(sim.max_tracking_error(200, 3200, 5), 1e-4);
  EXPECT_EQ(sim.pipeline.counters().stalls, 0u);
  EXPECT_EQ(sim.pipeline.counters().stream_starts, 1u);
  EXPECT_EQ(sim.pipeline.state(), State::kHolding);
  // The robot integrates the velocities: it ends at the goal of the client's trajectory
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(sim.pipeline.position()[i], sim.trajectory.goal[i], 1e-4) << "joint " << i;
    EXPECT_EQ(sim.pipeline.command()[i], 0.0);
  }
}

TEST(JointVelocityPipeline, StopsSmoothlyWhenStreamStalls) {
  StreamSimulation sim(default_config(JointVelocityPipeline::Mode::kExternal));
  sim.run(2500, 1600, 1.0, 0.0, 0.0);  // client dies in the middle of the motion
  expect_velocities_within_limits(sim.commands, sim.pipeline.config().limits);
  EXPECT_EQ(sim.pipeline.counters().stalls, 1u);
  EXPECT_EQ(sim.pipeline.state(), State::kHolding);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_EQ(sim.pipeline.command()[i], 0.0);
  }
}

TEST(JointVelocityPipeline, AbsorbsVelocityStep) {
  // A stream starting at full speed: ramped within the acceleration and jerk limits
  auto config = default_config(JointVelocityPipeline::Mode::kExternal);
  JointVelocityPipeline pipeline;
  pipeline.configure(config);
  pipeline.reset(ready_pose(), JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{JointArray{}, JointArray{}};
  JointArray step{};
  step[3] = 1.0;
  for (int k = 1; k <= 1000; ++k) {
    const int64_t now = kStart + k * kCycleNs;
    pipeline.push_sample({now, step});
    pipeline.step(now);
    commands.push_back(pipeline.command());
  }
  expect_velocities_within_limits(commands, config.limits);
  EXPECT_GT(pipeline.counters().limited_cycles, 0u);
  EXPECT_NEAR(pipeline.command()[3], 1.0, 1e-9);
}

TEST(JointVelocityPipeline, FinishesOnVelocityBeyondLimitsWithoutClamp) {
  auto config = default_config(JointVelocityPipeline::Mode::kExternal);
  config.clamp = false;
  JointVelocityPipeline pipeline;
  pipeline.configure(config);
  pipeline.reset(ready_pose(), JointArray{}, JointArray{}, kStart);
  JointArray too_fast{};
  too_fast[0] = 3.0;
  int64_t now = kStart;
  for (int k = 0; k < 100; ++k) {
    now += kCycleNs;
    pipeline.push_sample({now, too_fast});
    pipeline.step(now);
  }
  EXPECT_EQ(pipeline.state(), State::kFinished);
  EXPECT_EQ(pipeline.finish_reason(), FinishReason::kLimits);
}

TEST(JointVelocityPipeline, ExecutesInternalMotion) {
  auto config = default_config(JointVelocityPipeline::Mode::kInternal);
  JointVelocityPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{JointArray{}, JointArray{}};

  JointArray goal = start;
  goal[0] += 0.3;
  goal[6] -= 0.5;
  pipeline.request(quintic_request(1, goal, 1.5));
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 1; });

  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(pipeline.position()[i], goal[i], 1e-6) << "joint " << i;
  }
  expect_velocities_within_limits(commands, config.limits);
}

TEST(JointVelocityPipeline, CosineBumpReturnsToStart) {
  auto config = default_config(JointVelocityPipeline::Mode::kInternal);
  JointVelocityPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{JointArray{}, JointArray{}};

  JointMotionRequest request;
  request.id = 4;
  request.motion.type = JointMotion::Type::kCosineBump;
  request.motion.joint = 0;
  request.motion.amplitude = 2 * M_PI / 30;
  request.motion.duration = 10.0;
  pipeline.request(request);
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 4; });

  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_NEAR(pipeline.position()[0], start[0], 1e-6);
  expect_velocities_within_limits(commands, config.limits);
}

TEST(JointVelocityPipeline, RejectsFreeMotion) {
  // Gravity compensation exists only for the torque bridge
  JointVelocityPipeline pipeline;
  pipeline.configure(default_config(JointVelocityPipeline::Mode::kInternal));
  pipeline.reset(ready_pose(), JointArray{}, JointArray{}, kStart);
  JointMotionRequest request;
  request.id = 2;
  request.motion.type = JointMotion::Type::kFree;
  pipeline.request(request);
  std::vector<JointArray> commands;
  int64_t now = kStart;
  run_until(pipeline, commands, now, [] { return false; }, 10);
  EXPECT_EQ(pipeline.motion_status().last_finished_id, 2u);
  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kAborted);
  EXPECT_EQ(pipeline.state(), State::kHolding);
}

TEST(JointVelocityPipeline, StreamEndingAtRestIsNotAStall) {
  // The stream stops exactly at the end of the quintic, where the velocity is zero but its
  // derivatives are not. The gap is bridged at constant acceleration.
  auto config = default_config(JointVelocityPipeline::Mode::kExternal);
  JointVelocityPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  JointMotion trajectory;
  trajectory.start = start;
  trajectory.goal = start;
  trajectory.goal[0] += 0.6;
  trajectory.goal[5] -= 0.4;
  trajectory.duration = 3.0;
  std::vector<JointArray> commands{JointArray{}, JointArray{}};
  int64_t now = kStart;
  for (int k = 1; k <= 4000; ++k) {
    now += kCycleNs;
    if (k <= 3000) {
      JointArray p, v, a;
      trajectory.evaluate(k * 1e-3, p, v, a);
      pipeline.push_sample({now, v});
    }
    pipeline.step(now);
    commands.push_back(pipeline.command());
  }
  expect_velocities_within_limits(commands, config.limits);
  EXPECT_EQ(pipeline.counters().stalls, 0u);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(pipeline.position()[i], trajectory.goal[i], 1e-4) << "joint " << i;
  }
}
