#include <gtest/gtest.h>

#include <functional>
#include <vector>

#include "kinematics_checks.hpp"
#include "panda_utils/bridge/pipeline.hpp"

using panda_bridge::JointArray;
using panda_bridge::JointFeedback;
using panda_bridge::JointMotion;
using panda_bridge::JointMotionRequest;
using panda_bridge::JointTorquePipeline;
using panda_bridge::kCycleNs;
using panda_bridge::kCycleTime;
using panda_bridge::kNumJoints;
using panda_bridge::test::ready_pose;

namespace {

using State = JointTorquePipeline::State;
using Outcome = JointTorquePipeline::Outcome;
using FinishReason = JointTorquePipeline::FinishReason;

constexpr int64_t kStart = 1'000'000'000'000LL;  // arbitrary wall clock origin

JointTorquePipeline::Config default_config(JointTorquePipeline::Mode mode) {
  JointTorquePipeline::Config config;
  config.mode = mode;
  config.limits = panda_bridge::fer_joint_limits(0.9, 0.8, 0.5);
  config.torque = panda_bridge::fer_torque_settings(0.9, 0.5);
  return config;
}

// Gravity-compensated arm as the torque bridge sees it: one decoupled inertia per joint.
struct Plant {
  JointArray q{ready_pose()};
  JointArray dq{};
  JointArray inertia{1.0, 1.0, 0.5, 0.5, 0.2, 0.2, 0.1};

  void step(const JointArray &tau, const JointArray &external = JointArray{}) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      dq[i] += (tau[i] + external[i]) / inertia[i] * kCycleTime;
      q[i] += dq[i] * kCycleTime;
    }
  }

  JointFeedback feedback() const {
    JointFeedback robot;
    robot.q = robot.q_d = q;
    robot.dq = robot.dq_d = dq;
    return robot;
  }
};

// Closed loop of pipeline and plant, one cycle per call of step().
struct Loop {
  JointTorquePipeline pipeline;
  Plant plant;
  int64_t now{kStart};
  std::vector<JointArray> torques;

  explicit Loop(const JointTorquePipeline::Config &config) {
    pipeline.configure(config);
    pipeline.reset(plant.feedback(), now);
    torques.push_back(pipeline.command());
  }

  void step(const JointArray &external = JointArray{}) {
    now += kCycleNs;
    pipeline.set_feedback(plant.feedback());
    pipeline.step(now);
    plant.step(pipeline.command(), external);
    torques.push_back(pipeline.command());
  }

  void run(int cycles, const std::function<void(int)> &before_step = nullptr) {
    for (int k = 0; k < cycles; ++k) {
      if (before_step) {
        before_step(k);
      }
      step();
    }
  }

  int run_until(const std::function<bool()> &done, int max_cycles = 20000) {
    int cycles = 0;
    while (!done() && cycles < max_cycles) {
      step();
      ++cycles;
    }
    return cycles;
  }

  // Streams the given torques, stamped at the current cycle
  void push(const JointArray &tau) { pipeline.push_sample({now + kCycleNs, tau}); }
};

// Every change of the commanded torque within the rate limit.
void expect_rate_limited(const std::vector<JointArray> &torques, const panda_bridge::TorqueSettings &settings) {
  for (std::size_t k = 1; k < torques.size(); ++k) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      ASSERT_LE(std::abs(torques[k][i] - torques[k - 1][i]), settings.rate_max[i] * kCycleTime * (1 + 1e-9))
          << "joint " << i << " cycle " << k;
      ASSERT_LE(std::abs(torques[k][i]), settings.tau_max[i] * (1 + 1e-9)) << "joint " << i << " cycle " << k;
    }
  }
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

TEST(JointTorquePipeline, HoldsWhereTheRobotIs) {
  Loop loop(default_config(JointTorquePipeline::Mode::kExternal));
  const JointArray start = loop.plant.q;
  loop.run(1000);
  EXPECT_EQ(loop.pipeline.state(), State::kHolding);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(loop.plant.q[i], start[i], 1e-12);
    EXPECT_NEAR(loop.pipeline.command()[i], 0.0, 1e-12);
  }
}

TEST(JointTorquePipeline, HoldPullsBackAfterAPush) {
  Loop loop(default_config(JointTorquePipeline::Mode::kExternal));
  const JointArray start = loop.plant.q;
  JointArray push{};
  push[0] = 5.0;
  for (int k = 0; k < 300; ++k) {
    loop.step(push);
  }
  EXPECT_GT(loop.plant.q[0] - start[0], 1e-3);
  loop.run(3000);
  EXPECT_NEAR(loop.plant.q[0], start[0], 1e-4);
  expect_rate_limited(loop.torques, loop.pipeline.config().torque);
}

TEST(JointTorquePipeline, FollowsTorqueStreamWithinRateAndLimits) {
  Loop loop(default_config(JointTorquePipeline::Mode::kExternal));
  JointArray tau{};
  tau[4] = 100.0;  // far beyond the 12 Nm of joint 5
  tau[6] = 0.5;
  // Static robot: only the shaping of the torques is checked
  for (int k = 0; k < 200; ++k) {
    loop.push(tau);
    loop.now += kCycleNs;
    loop.pipeline.set_feedback(loop.plant.feedback());
    loop.pipeline.step(loop.now);
    loop.torques.push_back(loop.pipeline.command());
  }
  const auto &settings = loop.pipeline.config().torque;
  expect_rate_limited(loop.torques, settings);
  EXPECT_EQ(loop.pipeline.state(), State::kMoving);
  EXPECT_DOUBLE_EQ(loop.pipeline.command()[4], settings.tau_max[4]);
  EXPECT_DOUBLE_EQ(loop.pipeline.command()[6], 0.5);
  EXPECT_GT(loop.pipeline.counters().limited_cycles, 0u);
}

TEST(JointTorquePipeline, FinishesOnTorqueBeyondLimitsWithoutClamp) {
  auto config = default_config(JointTorquePipeline::Mode::kExternal);
  config.clamp = false;
  Loop loop(config);
  JointArray tau{};
  tau[0] = 200.0;
  for (int k = 0; k < 20; ++k) {
    loop.push(tau);
    loop.step();
  }
  EXPECT_EQ(loop.pipeline.state(), State::kFinished);
  EXPECT_EQ(loop.pipeline.finish_reason(), FinishReason::kLimits);
}

TEST(JointTorquePipeline, DampsThenHoldsWhenStreamStalls) {
  Loop loop(default_config(JointTorquePipeline::Mode::kExternal));
  JointArray tau{};
  tau[0] = 2.0;
  // 300 ms of torque on joint 1, then the client dies with the joint moving
  loop.run(300, [&](int) { loop.push(tau); });
  ASSERT_GT(loop.plant.dq[0], 0.3);
  loop.run_until([&] { return loop.pipeline.state() == State::kHolding; }, 2000);
  EXPECT_EQ(loop.pipeline.state(), State::kHolding);
  EXPECT_EQ(loop.pipeline.counters().stalls, 1u);
  const JointArray stopped = loop.plant.q;
  loop.run(2000);
  EXPECT_NEAR(loop.plant.q[0], stopped[0], 1e-3);
  EXPECT_NEAR(loop.plant.dq[0], 0.0, 1e-4);
  expect_rate_limited(loop.torques, loop.pipeline.config().torque);
}

TEST(JointTorquePipeline, ExecutesInternalMotionWithImpedance) {
  Loop loop(default_config(JointTorquePipeline::Mode::kInternal));
  JointArray goal = loop.plant.q;
  goal[0] += 0.3;
  goal[3] -= 0.2;
  loop.pipeline.request(quintic_request(1, goal, 2.0));
  loop.run_until([&] { return loop.pipeline.motion_status().last_finished_id == 1; });
  EXPECT_EQ(loop.pipeline.motion_status().last_outcome, Outcome::kSucceeded);
  EXPECT_EQ(loop.pipeline.state(), State::kHolding);
  // The hold keeps the goal of the motion
  loop.run(1000);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(loop.plant.q[i], goal[i], 1e-3) << "joint " << i;
  }
  expect_rate_limited(loop.torques, loop.pipeline.config().torque);
}

TEST(JointTorquePipeline, GravityCompensationFloatsUntilStopped) {
  Loop loop(default_config(JointTorquePipeline::Mode::kInternal));
  JointMotionRequest request;
  request.id = 3;
  request.motion.type = JointMotion::Type::kFree;
  request.name.set("gravity_compensation");
  loop.pipeline.request(request);
  loop.run(10);
  EXPECT_EQ(loop.pipeline.state(), State::kMoving);
  EXPECT_EQ(loop.pipeline.motion_status().active_id, 3u);

  // Moved by hand: no spring pulls the joint back
  const JointArray start = loop.plant.q;
  JointArray hand{};
  hand[2] = 1.0;
  for (int k = 0; k < 200; ++k) {
    loop.step(hand);
  }
  loop.run(500);
  EXPECT_GT(loop.plant.q[2] - start[2], 0.03);
  EXPECT_NEAR(loop.pipeline.command()[2], 0.0, 1e-12);

  JointMotionRequest cancel;
  cancel.kind = JointMotionRequest::Kind::kCancel;
  cancel.id = 0;
  loop.pipeline.request(cancel);
  loop.run_until([&] { return loop.pipeline.state() == State::kHolding; }, 2000);
  EXPECT_EQ(loop.pipeline.state(), State::kHolding);
  EXPECT_EQ(loop.pipeline.motion_status().last_outcome, Outcome::kCanceled);
  const JointArray stopped = loop.plant.q;
  loop.run(1000);
  EXPECT_NEAR(loop.plant.q[2], stopped[2], 1e-3);
  expect_rate_limited(loop.torques, loop.pipeline.config().torque);
}

TEST(JointTorquePipeline, FinishRequestDampsThenFinishes) {
  Loop loop(default_config(JointTorquePipeline::Mode::kInternal));
  JointArray goal = loop.plant.q;
  goal[1] += 0.4;
  loop.pipeline.request(quintic_request(5, goal, 1.5));
  loop.run(700);
  loop.pipeline.request_finish(FinishReason::kSafety);
  loop.run_until([&] { return loop.pipeline.state() == State::kFinished; }, 2000);
  EXPECT_EQ(loop.pipeline.state(), State::kFinished);
  EXPECT_EQ(loop.pipeline.motion_status().last_outcome, Outcome::kAborted);
  EXPECT_LT(std::abs(loop.plant.dq[1]), 0.02);
  expect_rate_limited(loop.torques, loop.pipeline.config().torque);
}
