#include <gtest/gtest.h>

#include <vector>

#include "kinematics_checks.hpp"
#include "panda_utils/bridge/joint_motions.hpp"
#include "panda_utils/bridge/joint_tracker.hpp"

using panda_bridge::JointArray;
using panda_bridge::JointMotion;
using panda_bridge::JointTracker;
using panda_bridge::kCycleTime;
using panda_bridge::kNumJoints;
using panda_bridge::test::expect_within_limits;
using panda_bridge::test::ready_pose;

namespace {

JointTracker make_tracker(const JointArray &start) {
  JointTracker tracker;
  tracker.set_limits(panda_bridge::fer_joint_limits(0.9, 0.8, 0.5), 0.01);
  tracker.reset(start, JointArray{}, JointArray{});
  return tracker;
}

}  // namespace

TEST(JointTracker, ReproducesFeasibleReference) {
  const JointArray start = ready_pose();
  JointMotion motion;
  motion.start = start;
  motion.goal = start;
  motion.goal[0] += 0.5;
  motion.goal[3] -= 0.3;
  motion.duration = 2.0;

  JointTracker tracker = make_tracker(start);
  double max_error = 0.0;
  // A few cycles past the end, so that the finite differences settle at rest
  for (int k = 1; k <= 2005; ++k) {
    JointArray p, v, a;
    motion.evaluate(k * kCycleTime, p, v, a);
    EXPECT_FALSE(tracker.track(p, v, a)) << "cycle " << k;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      max_error = std::max(max_error, std::abs(tracker.position()[i] - p[i]));
    }
  }
  EXPECT_LT(max_error, 1e-12);
  EXPECT_TRUE(tracker.at_rest());
}

TEST(JointTracker, SmoothsStepWithinLimits) {
  const JointArray start = ready_pose();
  JointTracker tracker = make_tracker(start);
  JointArray target = start;
  target[1] += 0.2;  // joint 2 has the lowest acceleration and jerk limits

  std::vector<JointArray> commands{start, start, start};
  for (int k = 0; k < 3000; ++k) {
    tracker.track(target, JointArray{}, JointArray{});
    commands.push_back(tracker.position());
  }
  expect_within_limits(commands, tracker.limits());
  EXPECT_NEAR(tracker.position()[1], target[1], 1e-9);
  EXPECT_TRUE(tracker.at_rest());
}

TEST(JointTracker, BrakesToRestWithinLimits) {
  const JointArray start = ready_pose();
  JointTracker tracker = make_tracker(start);
  JointArray velocity;
  velocity.fill(1.0);
  tracker.reset(start, velocity, JointArray{});

  // Reconstruct the previous commands from the initial velocity
  std::vector<JointArray> commands;
  for (int k = 3; k >= 1; --k) {
    JointArray p = start;
    for (auto &q : p) {
      q -= k * kCycleTime * 1.0;
    }
    commands.push_back(p);
  }
  commands.push_back(start);

  int cycles = 0;
  while (!tracker.brake() && cycles < 5000) {
    commands.push_back(tracker.position());
    ++cycles;
  }
  EXPECT_LT(cycles, 5000);
  EXPECT_TRUE(tracker.at_rest());
  expect_within_limits(commands, tracker.limits());
}

TEST(JointTracker, StopsBeforeJointLimits) {
  JointArray start = ready_pose();
  start[0] = 2.0;
  JointTracker tracker = make_tracker(start);
  // Target far beyond the upper limit of joint 1, with a velocity pushing towards it
  JointArray target = start, target_velocity{};
  target[0] = 10.0;
  target_velocity[0] = 2.0;

  std::vector<JointArray> commands{start, start, start};
  for (int k = 0; k < 4000; ++k) {
    tracker.track(target, target_velocity, JointArray{});
    commands.push_back(tracker.position());
  }
  const auto &limits = tracker.limits();
  EXPECT_LE(tracker.position()[0], limits.q_max[0] - 0.01 + 1e-9);
  EXPECT_GT(tracker.position()[0], limits.q_max[0] - 0.02);
  EXPECT_TRUE(tracker.at_rest());
  expect_within_limits(commands, limits);
}
