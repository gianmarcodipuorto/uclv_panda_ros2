#include <gtest/gtest.h>

#include <vector>

#include "kinematics_checks.hpp"
#include "panda_utils/bridge/joint_motions.hpp"
#include "panda_utils/bridge/joint_velocity_tracker.hpp"

using panda_bridge::JointArray;
using panda_bridge::JointMotion;
using panda_bridge::JointVelocityTracker;
using panda_bridge::kCycleTime;
using panda_bridge::kNumJoints;
using panda_bridge::test::expect_velocities_within_limits;
using panda_bridge::test::ready_pose;

namespace {

JointVelocityTracker make_tracker(const JointArray &start) {
  JointVelocityTracker tracker;
  tracker.set_limits(panda_bridge::fer_joint_limits(0.9, 0.8, 0.5), 0.01);
  tracker.reset(start, JointArray{}, JointArray{});
  return tracker;
}

}  // namespace

TEST(JointVelocityTracker, ReproducesFeasibleReference) {
  const JointArray start = ready_pose();
  JointMotion motion;
  motion.start = start;
  motion.goal = start;
  motion.goal[0] += 0.5;
  motion.goal[3] -= 0.3;
  motion.duration = 2.0;

  JointVelocityTracker tracker = make_tracker(start);
  double max_error = 0.0;
  for (int k = 1; k <= 2005; ++k) {
    JointArray p, v, a;
    motion.evaluate(k * kCycleTime, p, v, a);
    EXPECT_FALSE(tracker.track(v, a)) << "cycle " << k;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      max_error = std::max(max_error, std::abs(tracker.velocity()[i] - v[i]));
    }
  }
  EXPECT_LT(max_error, 1e-12);
  EXPECT_TRUE(tracker.at_rest());
  // The integrated command ends at the goal, up to the rectangle rule
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    EXPECT_NEAR(tracker.position()[i], motion.goal[i], 1e-6) << "joint " << i;
  }
}

TEST(JointVelocityTracker, SmoothsStepWithinLimits) {
  const JointArray start = ready_pose();
  JointVelocityTracker tracker = make_tracker(start);
  JointArray target{};
  target[1] = 0.5;  // joint 2 has the lowest acceleration and jerk limits

  std::vector<JointArray> commands{JointArray{}, JointArray{}};
  for (int k = 0; k < 1000; ++k) {
    tracker.track(target, JointArray{});
    commands.push_back(tracker.velocity());
  }
  expect_velocities_within_limits(commands, tracker.limits());
  EXPECT_NEAR(tracker.velocity()[1], target[1], 1e-9);
  // Converged: the reference is commanded as it is again
  EXPECT_FALSE(tracker.track(target, JointArray{}));
}

TEST(JointVelocityTracker, BrakesToRestWithinLimits) {
  const JointArray start = ready_pose();
  JointVelocityTracker tracker = make_tracker(start);
  JointArray velocity;
  velocity.fill(1.0);
  tracker.reset(start, velocity, JointArray{});

  std::vector<JointArray> commands{velocity, velocity};
  int cycles = 0;
  while (!tracker.brake() && cycles < 5000) {
    commands.push_back(tracker.velocity());
    ++cycles;
  }
  EXPECT_LT(cycles, 5000);
  EXPECT_TRUE(tracker.at_rest());
  expect_velocities_within_limits(commands, tracker.limits());
}

TEST(JointVelocityTracker, StopsBeforeJointLimits) {
  JointArray start = ready_pose();
  start[0] = 2.0;
  JointVelocityTracker tracker = make_tracker(start);
  // Full speed towards the upper limit of joint 1
  JointArray target{};
  target[0] = 2.0;

  std::vector<JointArray> commands{JointArray{}, JointArray{}};
  for (int k = 0; k < 4000; ++k) {
    tracker.track(target, JointArray{});
    commands.push_back(tracker.velocity());
  }
  const auto &limits = tracker.limits();
  EXPECT_LE(tracker.position()[0], limits.q_max[0] - 0.01 + 1e-9);
  EXPECT_GT(tracker.position()[0], limits.q_max[0] - 0.02);
  EXPECT_NEAR(tracker.velocity()[0], 0.0, 1e-3);
  expect_velocities_within_limits(commands, limits);
}

TEST(JointVelocityTracker, MotionCorrectionReachesThePosition) {
  const JointArray start = ready_pose();
  JointVelocityTracker tracker = make_tracker(start);
  // A reference position 1 mrad away, at rest: the correction closes the gap within the limits
  JointArray goal = start;
  goal[2] += 1e-3;
  std::vector<JointArray> commands{JointArray{}, JointArray{}};
  for (int k = 0; k < 1000; ++k) {
    tracker.follow_motion(goal, JointArray{}, JointArray{});
    commands.push_back(tracker.velocity());
  }
  expect_velocities_within_limits(commands, tracker.limits());
  EXPECT_NEAR(tracker.position()[2], goal[2], 1e-7);
  EXPECT_TRUE(tracker.at_rest());
}
