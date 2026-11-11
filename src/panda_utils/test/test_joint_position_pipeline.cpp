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
using panda_bridge::JointPositionPipeline;
using panda_bridge::kCycleNs;
using panda_bridge::kNumJoints;
using panda_bridge::test::expect_within_limits;
using panda_bridge::test::ready_pose;

namespace {

using State = JointPositionPipeline::State;
using Outcome = JointPositionPipeline::Outcome;
using FinishReason = JointPositionPipeline::FinishReason;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kStart = 1'000'000'000'000LL;  // arbitrary wall clock origin

JointPositionPipeline::Config default_config(JointPositionPipeline::Mode mode) {
  JointPositionPipeline::Config config;
  config.mode = mode;
  config.limits = panda_bridge::fer_joint_limits(0.9, 0.8, 0.5);
  return config;
}

// Simulated client: a quintic from start to goal sampled every period_ms, delivered to the
// pipeline with random latency, losses and duplicates. The robot clock runs at 1 kHz.
struct StreamSimulation {
  JointPositionPipeline pipeline;
  std::vector<JointArray> commands;
  JointMotion trajectory;
  std::mt19937 rng{42};

  struct InFlight {
    int64_t delivery_ns;
    JointPositionPipeline::Sample sample;
  };
  std::deque<InFlight> in_flight;

  explicit StreamSimulation(const JointPositionPipeline::Config &config) {
    pipeline.configure(config);
    const JointArray start = ready_pose();
    pipeline.reset(start, JointArray{}, JointArray{}, kStart);
    commands = {start, start, start};
    trajectory.start = start;
    trajectory.goal = start;
    trajectory.goal[0] += 0.6;
    trajectory.goal[1] -= 0.3;
    trajectory.goal[5] += 0.4;
    trajectory.duration = 3.0;
  }

  // Runs the robot loop for duration_ms while the client publishes until publish_until_ms.
  void run(int duration_ms, int publish_until_ms, int period_ms, double max_latency_ms, double loss,
           double duplicates) {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    for (int k = 1; k <= duration_ms; ++k) {
      const int64_t now = kStart + k * kCycleNs;
      if (k <= publish_until_ms && k % period_ms == 0 && uniform(rng) >= loss) {
        JointPositionPipeline::Sample sample;
        sample.stamp_ns = now;
        JointArray v, a;
        trajectory.evaluate((k * 1e-3) - 0.1, sample.values, v, a);  // client starts after 100 ms
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

  // Largest deviation from the trajectory delayed by the playout delay, over the given window.
  double max_tracking_error(int from_ms, int to_ms, int delay_ms) const {
    double error = 0.0;
    for (int k = from_ms; k < to_ms; ++k) {
      JointArray p, v, a;
      trajectory.evaluate((k - delay_ms) * 1e-3 - 0.1, p, v, a);
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        error = std::max(error, std::abs(commands[k + 2][i] - p[i]));
      }
    }
    return error;
  }
};

JointMotionRequest quintic_request(uint64_t id, const JointArray &goal, double duration) {
  JointMotionRequest request;
  request.id = id;
  request.motion.type = JointMotion::Type::kQuintic;
  request.motion.goal = goal;
  request.motion.duration = duration;
  request.name.set("test");
  return request;
}

int run_until(JointPositionPipeline &pipeline, std::vector<JointArray> &commands, int64_t &now,
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

}  // namespace

TEST(JointPositionPipeline, FollowsJitteryStreamWithinLimits) {
  StreamSimulation sim(default_config(JointPositionPipeline::Mode::kExternal));
  // 1 kHz stream, up to 4 ms of latency (beyond the 5 ms playout delay would starve), 5% losses
  // and 5% duplicates
  sim.run(4000, 3200, 1, 4.0, 0.05, 0.05);
  expect_within_limits(sim.commands, sim.pipeline.config().limits);
  EXPECT_LT(sim.max_tracking_error(200, 3200, 5), 1e-4);
  EXPECT_EQ(sim.pipeline.counters().stalls, 0u);
  EXPECT_EQ(sim.pipeline.counters().stream_starts, 1u);
  EXPECT_EQ(sim.pipeline.state(), State::kHolding);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(sim.pipeline.command()[i], sim.trajectory.goal[i], 1e-6);
  }
}

TEST(JointPositionPipeline, FollowsSlowStreamWithinLimits) {
  StreamSimulation sim(default_config(JointPositionPipeline::Mode::kExternal));
  // 100 Hz stream: gaps longer than the playout delay are bridged by extrapolation
  sim.run(4000, 3200, 10, 2.0, 0.0, 0.0);
  expect_within_limits(sim.commands, sim.pipeline.config().limits);
  EXPECT_LT(sim.max_tracking_error(200, 3200, 5), 5e-3);
  EXPECT_EQ(sim.pipeline.counters().stalls, 0u);
}

TEST(JointPositionPipeline, StopsSmoothlyWhenStreamStalls) {
  StreamSimulation sim(default_config(JointPositionPipeline::Mode::kExternal));
  sim.run(2500, 1600, 1, 1.0, 0.0, 0.0);  // client dies in the middle of the motion
  expect_within_limits(sim.commands, sim.pipeline.config().limits);
  EXPECT_EQ(sim.pipeline.counters().stalls, 1u);
  EXPECT_EQ(sim.pipeline.state(), State::kHolding);
}

TEST(JointPositionPipeline, ResumesAfterStall) {
  auto config = default_config(JointPositionPipeline::Mode::kExternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{start, start, start};

  int64_t now = kStart;
  // Open-loop ramp on joint 1 starting from the current command
  auto stream_ramp = [&](int cycles, double slope) {
    const double base = pipeline.command()[0];
    const int64_t t0 = now;
    for (int k = 0; k < cycles; ++k) {
      now += kCycleNs;
      JointPositionPipeline::Sample sample;
      sample.stamp_ns = now;
      sample.values = start;
      sample.values[0] = base + slope * panda_bridge::ns_to_s(now - t0);
      pipeline.push_sample(sample);
      pipeline.step(now);
      commands.push_back(pipeline.command());
    }
  };
  stream_ramp(500, 0.2);
  run_until(pipeline, commands, now, [&] { return pipeline.state() == State::kHolding; });
  EXPECT_EQ(pipeline.counters().stalls, 1u);
  stream_ramp(500, 0.2);
  EXPECT_EQ(pipeline.counters().stream_starts, 2u);
  expect_within_limits(commands, config.limits);
}

TEST(JointPositionPipeline, RejectsStreamFarFromCommand) {
  JointPositionPipeline pipeline;
  pipeline.configure(default_config(JointPositionPipeline::Mode::kExternal));
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  int64_t now = kStart;
  for (int k = 0; k < 100; ++k) {
    now += kCycleNs;
    JointPositionPipeline::Sample sample;
    sample.stamp_ns = now;
    sample.values = start;
    sample.values[2] += 0.5;
    pipeline.push_sample(sample);
    pipeline.step(now);
  }
  EXPECT_GT(pipeline.counters().rejected_jumps, 0u);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_EQ(pipeline.command(), start);
}

TEST(JointPositionPipeline, IgnoresNonFiniteSamples) {
  JointPositionPipeline pipeline;
  pipeline.configure(default_config(JointPositionPipeline::Mode::kExternal));
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  int64_t now = kStart;
  for (int k = 0; k < 50; ++k) {
    now += kCycleNs;
    JointPositionPipeline::Sample sample;
    sample.stamp_ns = now;
    sample.values = start;
    sample.values[4] = std::nan("");
    pipeline.push_sample(sample);
    pipeline.step(now);
  }
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_EQ(pipeline.command(), start);
}

TEST(JointPositionPipeline, FinishesAfterCommandTimeout) {
  auto config = default_config(JointPositionPipeline::Mode::kExternal);
  config.command_timeout_ns = 100 * kMs;
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  pipeline.reset(ready_pose(), JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands;
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.state() == State::kFinished; }, 200);
  EXPECT_EQ(pipeline.state(), State::kFinished);
  EXPECT_EQ(pipeline.finish_reason(), FinishReason::kCommandTimeout);
}

TEST(JointPositionPipeline, ExecutesInternalMotion) {
  auto config = default_config(JointPositionPipeline::Mode::kInternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{start, start, start};

  JointArray goal = start;
  goal[0] += 0.3;
  goal[6] -= 0.5;
  pipeline.request(quintic_request(1, goal, 1.5));
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 1; });

  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_GE(commands.size(), 1500u);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(pipeline.command()[i], goal[i], 1e-9);
  }
  expect_within_limits(commands, config.limits);
}

TEST(JointPositionPipeline, StretchesTooFastMotion) {
  auto config = default_config(JointPositionPipeline::Mode::kInternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{start, start, start};

  JointArray goal = start;
  goal[0] += 1.5;
  pipeline.request(quintic_request(7, goal, 0.1));  // far too fast for the limits
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 7; });
  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  expect_within_limits(commands, config.limits);
}

TEST(JointPositionPipeline, CancelStopsMotion) {
  auto config = default_config(JointPositionPipeline::Mode::kInternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{start, start, start};

  JointArray goal = start;
  goal[3] += 0.8;
  pipeline.request(quintic_request(3, goal, 2.0));
  int64_t now = kStart;
  run_until(pipeline, commands, now, [] { return false; }, 1000);
  JointMotionRequest cancel;
  cancel.kind = JointMotionRequest::Kind::kCancel;
  cancel.id = 3;
  pipeline.request(cancel);
  run_until(pipeline, commands, now, [&] { return pipeline.state() == State::kHolding; });

  EXPECT_EQ(pipeline.motion_status().last_finished_id, 3u);
  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kCanceled);
  EXPECT_LT(pipeline.command()[3], goal[3] - 0.05);
  expect_within_limits(commands, config.limits);
}

TEST(JointPositionPipeline, NewMotionPreemptsRunningOne) {
  auto config = default_config(JointPositionPipeline::Mode::kInternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{start, start, start};

  JointArray first = start, second = start;
  first[0] += 0.8;
  second[0] -= 0.3;
  pipeline.request(quintic_request(1, first, 2.0));
  int64_t now = kStart;
  run_until(pipeline, commands, now, [] { return false; }, 800);
  pipeline.request(quintic_request(2, second, 1.0));
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 2; });

  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_NEAR(pipeline.command()[0], second[0], 1e-9);
  expect_within_limits(commands, config.limits);
}

TEST(JointPositionPipeline, FinishRequestStopsThenFinishes) {
  auto config = default_config(JointPositionPipeline::Mode::kInternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{start, start, start};

  JointArray goal = start;
  goal[1] += 0.5;
  pipeline.request(quintic_request(5, goal, 1.0));
  int64_t now = kStart;
  run_until(pipeline, commands, now, [] { return false; }, 500);
  pipeline.request_finish(FinishReason::kRequested);
  run_until(pipeline, commands, now, [&] { return pipeline.state() == State::kFinished; });

  EXPECT_EQ(pipeline.state(), State::kFinished);
  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kAborted);
  EXPECT_EQ(pipeline.finish_reason(), FinishReason::kRequested);
  expect_within_limits(commands, config.limits);
}

TEST(JointPositionPipeline, CosineBumpReturnsToStart) {
  auto config = default_config(JointPositionPipeline::Mode::kInternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray start = ready_pose();
  pipeline.reset(start, JointArray{}, JointArray{}, kStart);
  std::vector<JointArray> commands{start, start, start};

  JointMotionRequest request;
  request.id = 9;
  request.motion.type = JointMotion::Type::kCosineBump;
  request.motion.joint = 0;
  request.motion.amplitude = 2 * M_PI / 30;
  request.motion.duration = 10.0;
  pipeline.request(request);
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 9; });

  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_NEAR(pipeline.command()[0], start[0], 1e-9);
  expect_within_limits(commands, config.limits);
}

TEST(JointPositionPipeline, AbsorbsStreamStartingFromMeasuredPosition) {
  // The client starts from the measured q, 1 mrad from the commanded q_d: the step is absorbed
  // within the limits and the command reaches the end of the stream
  auto config = default_config(JointPositionPipeline::Mode::kExternal);
  JointPositionPipeline pipeline;
  pipeline.configure(config);
  const JointArray q = ready_pose();
  JointArray q_d = q;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    q_d[i] += (i % 2) ? 1e-3 : -1e-3;
  }
  pipeline.reset(q_d, JointArray{}, JointArray{}, kStart);
  JointMotion trajectory;
  trajectory.start = q;
  trajectory.goal = q;
  trajectory.goal[0] -= 0.2;  // as simple_joints_traj: 0.2 rad in 2 s
  trajectory.duration = 2.0;

  std::vector<JointArray> commands{q_d, q_d, q_d};
  for (int k = 1; k <= 2500; ++k) {
    const int64_t now = kStart + k * kCycleNs;
    JointPositionPipeline::Sample sample;
    sample.stamp_ns = now;
    JointArray v, a;
    trajectory.evaluate(k * 1e-3, sample.values, v, a);
    if (k <= 2100) {
      pipeline.push_sample(sample);
    }
    pipeline.step(now);
    commands.push_back(pipeline.command());
  }
  expect_within_limits(commands, config.limits);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_EQ(pipeline.counters().stalls, 0u);
  EXPECT_EQ(pipeline.counters().extrapolated_cycles, 0u);
  EXPECT_GE(pipeline.counters().playout_margin_min_ns, 0);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(pipeline.command()[i], trajectory.goal[i], 1e-4);
  }
}
