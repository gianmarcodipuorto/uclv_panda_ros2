#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <random>
#include <vector>

#include "kinematics_checks.hpp"
#include "panda_utils/bridge/pipeline.hpp"

using panda_bridge::CartesianFeedback;
using panda_bridge::CartesianMotion;
using panda_bridge::CartesianMotionRequest;
using panda_bridge::CartesianPosePipeline;
using panda_bridge::CartesianState;
using panda_bridge::CartesianVelocityPipeline;
using panda_bridge::kCycleNs;
using panda_bridge::Pose;
using panda_bridge::test::expect_cartesian_within_limits;
using panda_bridge::test::ready_ee_pose;

namespace {

using State = panda_bridge::PipelineTypes::State;
using Mode = panda_bridge::PipelineTypes::Mode;
using Outcome = panda_bridge::PipelineTypes::Outcome;
using Config = panda_bridge::PipelineTypes::Config;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kStart = 1'000'000'000'000LL;  // arbitrary wall clock origin

Config default_config(Mode mode) {
  Config config;
  config.mode = mode;
  config.cartesian.limits = panda_bridge::fer_cartesian_limits(0.6, 0.3, 0.3);
  config.cartesian.min_height = 0.11;
  return config;
}

CartesianFeedback at_rest(const Pose &pose) {
  CartesianFeedback robot;
  robot.O_T_EE_c = panda_bridge::matrix_from_pose(pose);
  robot.elbow_c = {0.0, -1.0};
  return robot;
}

CartesianMotion test_motion(const Pose &start) {
  CartesianMotion motion;
  motion.start = start;
  motion.goal = start;
  motion.goal.p = {start.p[0] + 0.1, start.p[1] - 0.05, start.p[2] + 0.08};
  motion.goal.q = panda_bridge::rotated(start.q, Eigen::Vector3d(0.1, 0.3, -0.2));
  motion.duration = 3.0;
  return motion;
}

double distance(const Pose &a, const Pose &b) {
  return (panda_bridge::to_eigen(a.p) - panda_bridge::to_eigen(b.p)).norm() +
         panda_bridge::rotation_between(a.q, b.q).norm();
}

std::array<double, 7> pose_values(const Pose &pose) {
  return {pose.p[0], pose.p[1], pose.p[2], pose.q[0], pose.q[1], pose.q[2], pose.q[3]};
}

// Simulated client of the pose bridge: the poses of a motion, sampled every millisecond and
// delivered with random latency, losses and duplicates; optionally with the quaternion sign
// flipped at every other sample, as clients that normalize w >= 0 may do.
struct PoseStream {
  CartesianPosePipeline pipeline;
  std::vector<CartesianState> commands;
  CartesianMotion trajectory;
  std::mt19937 rng{11};

  struct InFlight {
    int64_t delivery_ns;
    CartesianPosePipeline::Sample sample;
  };
  std::deque<InFlight> in_flight;

  explicit PoseStream(const Config &config) {
    pipeline.configure(config);
    pipeline.reset(at_rest(ready_ee_pose()), kStart);
    commands = {pipeline.command(), pipeline.command(), pipeline.command()};
    trajectory = test_motion(ready_ee_pose());
  }

  void run(int duration_ms, int publish_until_ms, double max_latency_ms, double loss, double duplicates,
           bool flip_signs = false) {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    for (int k = 1; k <= duration_ms; ++k) {
      const int64_t now = kStart + k * kCycleNs;
      if (k <= publish_until_ms && uniform(rng) >= loss) {
        CartesianState point;
        trajectory.evaluate(k * 1e-3 - 0.1, point);  // client starts after 100 ms
        CartesianPosePipeline::Sample sample;
        sample.stamp_ns = now;
        sample.values = pose_values(point.pose);
        if (flip_signs && k % 2 == 0) {
          for (std::size_t i = 3; i < 7; ++i) {
            sample.values[i] = -sample.values[i];
          }
        }
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

  // Largest deviation from the trajectory delayed by the playout delay.
  double max_tracking_error(int from_ms, int to_ms, int delay_ms) const {
    double error = 0.0;
    for (int k = from_ms; k < to_ms; ++k) {
      CartesianState point;
      trajectory.evaluate((k - delay_ms) * 1e-3 - 0.1, point);
      error = std::max(error, distance(commands[k + 2].pose, point.pose));
    }
    return error;
  }
};

template <typename Pipeline>
int run_until(Pipeline &pipeline, std::vector<CartesianState> &commands, int64_t &now,
              const std::function<bool()> &done, int max_cycles = 30000) {
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

TEST(CartesianPosePipeline, FollowsJitteryStreamWithinLimits) {
  PoseStream sim(default_config(Mode::kExternal));
  sim.run(4000, 3300, 4.0, 0.05, 0.05);
  expect_cartesian_within_limits(sim.commands, sim.pipeline.config().cartesian.limits, true);
  EXPECT_LT(sim.max_tracking_error(200, 3300, 5), 1e-5);
  EXPECT_EQ(sim.pipeline.counters().stalls, 0u);
  EXPECT_EQ(sim.pipeline.counters().stream_starts, 1u);
  EXPECT_EQ(sim.pipeline.state(), State::kHolding);
  EXPECT_LT(distance(sim.pipeline.command().pose, sim.trajectory.goal), 1e-6);
}

TEST(CartesianPosePipeline, QuaternionSignFlipsDoNotMatter) {
  PoseStream sim(default_config(Mode::kExternal));
  sim.run(4000, 3300, 1.0, 0.0, 0.0, true);
  expect_cartesian_within_limits(sim.commands, sim.pipeline.config().cartesian.limits, true);
  EXPECT_LT(sim.max_tracking_error(200, 3300, 5), 1e-5);
  EXPECT_LT(distance(sim.pipeline.command().pose, sim.trajectory.goal), 1e-6);
}

TEST(CartesianPosePipeline, StopsSmoothlyWhenStreamStalls) {
  PoseStream sim(default_config(Mode::kExternal));
  sim.run(3000, 1600, 1.0, 0.0, 0.0);  // client dies in the middle of the motion
  expect_cartesian_within_limits(sim.commands, sim.pipeline.config().cartesian.limits, true);
  EXPECT_EQ(sim.pipeline.counters().stalls, 1u);
  EXPECT_EQ(sim.pipeline.state(), State::kHolding);
}

TEST(CartesianPosePipeline, RejectsStreamFarFromCommand) {
  CartesianPosePipeline pipeline;
  pipeline.configure(default_config(Mode::kExternal));
  const Pose start = ready_ee_pose();
  pipeline.reset(at_rest(start), kStart);
  Pose far = start;
  far.p[0] += 0.2;
  int64_t now = kStart;
  for (int k = 0; k < 100; ++k) {
    now += kCycleNs;
    pipeline.push_sample({now, pose_values(far)});
    pipeline.step(now);
  }
  EXPECT_GT(pipeline.counters().rejected_jumps, 0u);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_LT(distance(pipeline.command().pose, start), 1e-12);
}

TEST(CartesianPosePipeline, ExecutesInternalMotion) {
  CartesianPosePipeline pipeline;
  const Config config = default_config(Mode::kInternal);
  pipeline.configure(config);
  const Pose start = ready_ee_pose();
  pipeline.reset(at_rest(start), kStart);
  std::vector<CartesianState> commands{pipeline.command(), pipeline.command(), pipeline.command()};
  CartesianMotionRequest request;
  request.id = 1;
  request.motion = test_motion(start);
  request.motion.duration = 1.0;
  pipeline.request(request);
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 1; });
  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_LT(distance(pipeline.command().pose, request.motion.goal), 1e-9);
  expect_cartesian_within_limits(commands, config.cartesian.limits, true);
}

TEST(CartesianPosePipeline, CircleReturnsToStart) {
  CartesianPosePipeline pipeline;
  const Config config = default_config(Mode::kInternal);
  pipeline.configure(config);
  const Pose start = ready_ee_pose();
  pipeline.reset(at_rest(start), kStart);
  std::vector<CartesianState> commands{pipeline.command(), pipeline.command(), pipeline.command()};
  CartesianMotionRequest request;
  request.id = 2;
  request.motion.type = CartesianMotion::Type::kCircle;
  request.motion.amplitude = 0.05;
  request.motion.duration = 1.0;  // stretched to the limits
  pipeline.request(request);
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 2; });
  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_LT(distance(pipeline.command().pose, start), 1e-9);
  expect_cartesian_within_limits(commands, config.cartesian.limits, true);
  // Half way the end effector is on the opposite side of the circle
  const auto farthest = std::max_element(commands.begin(), commands.end(), [&](const auto &a, const auto &b) {
    return std::abs(a.pose.p[0] - start.p[0]) < std::abs(b.pose.p[0] - start.p[0]);
  });
  EXPECT_NEAR(farthest->pose.p[0], start.p[0] - 0.1, 1e-3);
}

TEST(CartesianPosePipeline, StreamBelowMinHeightIsClamped) {
  CartesianPosePipeline pipeline;
  const Config config = default_config(Mode::kExternal);
  pipeline.configure(config);
  Pose start = ready_ee_pose();
  start.p[2] = 0.14;
  pipeline.reset(at_rest(start), kStart);
  std::vector<CartesianState> commands{pipeline.command(), pipeline.command(), pipeline.command()};
  // Down at 2 cm/s for 3 s: 6 cm, to 2 cm under the minimum height
  int64_t now = kStart;
  for (int k = 1; k <= 4000; ++k) {
    now += kCycleNs;
    Pose sample = start;
    sample.p[2] -= 0.02 * std::min(k, 3000) * 1e-3;
    pipeline.push_sample({now, pose_values(sample)});
    pipeline.step(now);
    commands.push_back(pipeline.command());
  }
  for (const auto &command : commands) {
    ASSERT_GE(command.pose.p[2], config.cartesian.min_height - 1e-9);
  }
  EXPECT_GT(pipeline.counters().limited_cycles, 0u);
  expect_cartesian_within_limits(commands, config.cartesian.limits, true);
}

TEST(CartesianVelocityPipeline, FollowsTwistStreamWithinLimits) {
  CartesianVelocityPipeline pipeline;
  const Config config = default_config(Mode::kExternal);
  pipeline.configure(config);
  const Pose start = ready_ee_pose();
  pipeline.reset(at_rest(start), kStart);
  const CartesianMotion trajectory = test_motion(start);
  std::vector<CartesianState> commands{pipeline.command(), pipeline.command()};
  int64_t now = kStart;
  for (int k = 1; k <= 4000; ++k) {
    now += kCycleNs;
    if (k <= 3300) {
      CartesianState point;
      trajectory.evaluate(k * 1e-3 - 0.1, point);
      pipeline.push_sample({now, {point.velocity[0], point.velocity[1], point.velocity[2], point.velocity[3],
                                  point.velocity[4], point.velocity[5]}});
    }
    pipeline.step(now);
    commands.push_back(pipeline.command());
  }
  expect_cartesian_within_limits(commands, config.cartesian.limits, false);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_EQ(pipeline.counters().stalls, 0u);
  // The robot integrates the twists: it ends at the goal of the client's trajectory
  EXPECT_LT(distance(pipeline.position().pose, trajectory.goal), 1e-4);
}

TEST(CartesianVelocityPipeline, ExecutesInternalMotion) {
  CartesianVelocityPipeline pipeline;
  const Config config = default_config(Mode::kInternal);
  pipeline.configure(config);
  const Pose start = ready_ee_pose();
  pipeline.reset(at_rest(start), kStart);
  std::vector<CartesianState> commands{pipeline.command(), pipeline.command()};
  CartesianMotionRequest request;
  request.id = 4;
  request.motion = test_motion(start);
  pipeline.request(request);
  int64_t now = kStart;
  run_until(pipeline, commands, now, [&] { return pipeline.motion_status().last_finished_id == 4; });
  EXPECT_EQ(pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_LT(distance(pipeline.position().pose, request.motion.goal), 1e-6);
  expect_cartesian_within_limits(commands, config.cartesian.limits, false);
}

TEST(CartesianVelocityPipeline, StreamEndingAtRestIsNotAStall) {
  // As simple_cartesian_traj --velocity: the stream stops exactly at the end of the quintic, where
  // the twist is zero but its derivatives are not. The gap is bridged at constant acceleration.
  CartesianVelocityPipeline pipeline;
  const Config config = default_config(Mode::kExternal);
  pipeline.configure(config);
  const Pose start = ready_ee_pose();
  pipeline.reset(at_rest(start), kStart);
  const CartesianMotion trajectory = test_motion(start);
  std::vector<CartesianState> commands{pipeline.command(), pipeline.command()};
  int64_t now = kStart;
  for (int k = 1; k <= 4000; ++k) {
    now += kCycleNs;
    if (k <= 3000) {
      CartesianState point;
      trajectory.evaluate(k * 1e-3, point);
      pipeline.push_sample({now, {point.velocity[0], point.velocity[1], point.velocity[2], point.velocity[3],
                                  point.velocity[4], point.velocity[5]}});
    }
    pipeline.step(now);
    commands.push_back(pipeline.command());
  }
  expect_cartesian_within_limits(commands, config.cartesian.limits, false);
  EXPECT_EQ(pipeline.counters().stalls, 0u);
  EXPECT_EQ(pipeline.state(), State::kHolding);
  EXPECT_LT(distance(pipeline.position().pose, trajectory.goal), 1e-4);
}
