#include <gtest/gtest.h>

#include <vector>

#include "kinematics_checks.hpp"
#include "panda_utils/bridge/cartesian_tracker.hpp"

using panda_bridge::CartesianArray;
using panda_bridge::CartesianFeedback;
using panda_bridge::CartesianMotion;
using panda_bridge::CartesianPoseTracker;
using panda_bridge::CartesianSettings;
using panda_bridge::CartesianState;
using panda_bridge::CartesianVelocityTracker;
using panda_bridge::kCycleTime;
using panda_bridge::Pose;
using panda_bridge::Quat;
using panda_bridge::test::expect_cartesian_within_limits;
using panda_bridge::test::ready_ee_pose;

namespace {

CartesianSettings test_settings(bool use_elbow = false) {
  CartesianSettings settings;
  settings.limits = panda_bridge::fer_cartesian_limits(0.6, 0.3, 0.3);
  settings.min_height = 0.11;
  settings.use_elbow = use_elbow;
  return settings;
}

template <typename Tracker>
Tracker make_tracker(const Pose &start, bool use_elbow = false, const CartesianArray &twist = CartesianArray{}) {
  Tracker tracker;
  tracker.set_settings(test_settings(use_elbow));
  CartesianFeedback robot;
  robot.O_T_EE_c = panda_bridge::matrix_from_pose(start);
  robot.elbow_c = {0.0, -1.0};
  for (std::size_t i = 0; i < 6; ++i) {
    robot.O_dP_EE_c[i] = twist[i];
  }
  tracker.reset(robot);
  return tracker;
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

}  // namespace

TEST(CartesianMath, RotationVectorRoundTrip) {
  for (const Eigen::Vector3d &r : {Eigen::Vector3d(0.3, -0.2, 0.5), Eigen::Vector3d(1e-9, 0.0, 0.0),
                                  Eigen::Vector3d(0.0, 0.0, M_PI - 1e-6)}) {
    const Eigen::Vector3d back = panda_bridge::rotation_vector(panda_bridge::from_rotation_vector(r));
    EXPECT_LT((back - r).norm(), 1e-9);
  }
}

TEST(CartesianMath, PoseMatrixRoundTrip) {
  Pose pose;
  pose.p = {0.3, -0.1, 0.5};
  pose.q = panda_bridge::normalized(Quat{0.3, -0.5, 0.1, 0.8});
  const Pose back = panda_bridge::pose_from_matrix(panda_bridge::matrix_from_pose(pose));
  EXPECT_LT(distance(pose, back), 1e-12);
}

TEST(CartesianMath, AngularVelocityOfQuaternionRate) {
  const Quat q0 = ready_ee_pose().q;
  const Eigen::Vector3d w(0.2, -0.4, 0.7);
  const double h = 1e-6;
  const Quat q1 = panda_bridge::rotated(q0, w * h);
  const Quat q_dot{(q1[0] - q0[0]) / h, (q1[1] - q0[1]) / h, (q1[2] - q0[2]) / h, (q1[3] - q0[3]) / h};
  EXPECT_LT((panda_bridge::angular_velocity(q0, q_dot) - w).norm(), 1e-5);
}

TEST(CartesianPoseTracker, ReproducesFeasibleMotion) {
  const Pose start = ready_ee_pose();
  auto tracker = make_tracker<CartesianPoseTracker>(start);
  const CartesianMotion motion = test_motion(start);
  double max_error = 0.0;
  for (int k = 1; k <= 3005; ++k) {
    CartesianState point;
    motion.evaluate(k * kCycleTime, point);
    EXPECT_FALSE(tracker.track_pose(point)) << "cycle " << k;
    max_error = std::max(max_error, distance(tracker.state().pose, point.pose));
  }
  EXPECT_LT(max_error, 1e-12);
  EXPECT_TRUE(tracker.at_rest());
}

TEST(CartesianPoseTracker, SmoothsStepWithinLimits) {
  const Pose start = ready_ee_pose();
  auto tracker = make_tracker<CartesianPoseTracker>(start);
  CartesianState target;
  target.pose = start;
  target.pose.p[1] += 0.03;
  target.pose.q = panda_bridge::rotated(start.q, Eigen::Vector3d(0.0, 0.0, 0.1));

  std::vector<CartesianState> commands{tracker.state(), tracker.state(), tracker.state()};
  for (int k = 0; k < 3000; ++k) {
    tracker.track_pose(target);
    commands.push_back(tracker.state());
  }
  expect_cartesian_within_limits(commands, tracker.settings().limits, true);
  EXPECT_LT(distance(tracker.state().pose, target.pose), 1e-9);
  EXPECT_TRUE(tracker.at_rest());
}

TEST(CartesianPoseTracker, StaysAboveMinHeight) {
  const Pose start = ready_ee_pose();
  auto tracker = make_tracker<CartesianPoseTracker>(start);
  CartesianState target;
  target.pose = start;
  target.pose.p[2] = -0.1;  // under the table

  std::vector<CartesianState> commands{tracker.state(), tracker.state(), tracker.state()};
  for (int k = 0; k < 5000; ++k) {
    tracker.track_pose(target);
    commands.push_back(tracker.state());
  }
  const double min_height = tracker.settings().min_height;
  for (const auto &command : commands) {
    ASSERT_GE(command.pose.p[2], min_height - 1e-9);
  }
  EXPECT_NEAR(tracker.state().pose.p[2], min_height, 1e-6);
  expect_cartesian_within_limits(commands, tracker.settings().limits, true);
}

TEST(CartesianPoseTracker, BrakesToRestWithinLimits) {
  const Pose start = ready_ee_pose();
  const CartesianArray twist{0.3, 0.0, 0.1, 0.0, 0.0, 0.5, 0.0};
  auto tracker = make_tracker<CartesianPoseTracker>(start, false, twist);
  // Previous poses consistent with the initial twist
  CartesianState before = tracker.state();
  std::vector<CartesianState> commands;
  for (int k = 2; k >= 1; --k) {
    CartesianState earlier = before;
    for (std::size_t i = 0; i < 3; ++i) {
      earlier.pose.p[i] -= twist[i] * k * kCycleTime;
    }
    earlier.pose.q = panda_bridge::rotated(before.pose.q, Eigen::Vector3d(twist[3], twist[4], twist[5]) * (-k * kCycleTime));
    commands.push_back(earlier);
  }
  commands.push_back(before);
  int cycles = 0;
  while (!tracker.brake() && cycles < 5000) {
    commands.push_back(tracker.state());
    ++cycles;
  }
  EXPECT_LT(cycles, 5000);
  expect_cartesian_within_limits(commands, tracker.settings().limits, true);
}

TEST(CartesianPoseTracker, MovesTheElbow) {
  const Pose start = ready_ee_pose();
  auto tracker = make_tracker<CartesianPoseTracker>(start, true);
  CartesianMotion motion = test_motion(start);
  motion.goal_elbow = 0.3;
  ASSERT_TRUE(tracker.prepare_motion(motion));
  std::vector<CartesianState> commands{tracker.state(), tracker.state(), tracker.state()};
  for (int k = 1; k <= 3100; ++k) {
    CartesianState point;
    motion.evaluate(k * kCycleTime, point);
    tracker.follow_motion(point);
    commands.push_back(tracker.state());
  }
  EXPECT_NEAR(tracker.state().elbow, 0.3, 1e-9);
  EXPECT_LT(distance(tracker.state().pose, motion.goal), 1e-9);
  expect_cartesian_within_limits(commands, tracker.settings().limits, true);
}

TEST(CartesianPoseTracker, RejectsMotionsBelowMinHeight) {
  const Pose start = ready_ee_pose();
  auto tracker = make_tracker<CartesianPoseTracker>(start);
  CartesianMotion motion = test_motion(start);
  motion.goal.p[2] = 0.05;
  EXPECT_FALSE(tracker.prepare_motion(motion));
  motion.type = CartesianMotion::Type::kLineBump;
  motion.axis = 2;
  motion.amplitude = -0.35;
  EXPECT_FALSE(tracker.prepare_motion(motion));
  motion.amplitude = 0.1;
  EXPECT_TRUE(tracker.prepare_motion(motion));
}

TEST(CartesianVelocityTracker, ReproducesFeasibleTwist) {
  const Pose start = ready_ee_pose();
  auto tracker = make_tracker<CartesianVelocityTracker>(start);
  const CartesianMotion motion = test_motion(start);
  double max_error = 0.0;
  for (int k = 1; k <= 3005; ++k) {
    CartesianState point;
    motion.evaluate(k * kCycleTime, point);
    EXPECT_FALSE(tracker.track_velocity(point.velocity, point.acceleration)) << "cycle " << k;
    for (std::size_t i = 0; i < 6; ++i) {
      max_error = std::max(max_error, std::abs(tracker.state().velocity[i] - point.velocity[i]));
    }
  }
  EXPECT_LT(max_error, 1e-12);
  // The integrated twist ends at the goal of the motion
  EXPECT_LT(distance(tracker.state().pose, motion.goal), 1e-6);
}

TEST(CartesianVelocityTracker, SmoothsTwistStepWithinLimits) {
  auto tracker = make_tracker<CartesianVelocityTracker>(ready_ee_pose());
  const CartesianArray twist{0.2, -0.1, 0.0, 0.0, 0.4, 0.0, 0.0};
  std::vector<CartesianState> commands{tracker.state(), tracker.state()};
  for (int k = 0; k < 1000; ++k) {
    tracker.track_velocity(twist, CartesianArray{});
    commands.push_back(tracker.state());
  }
  expect_cartesian_within_limits(commands, tracker.settings().limits, false);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(tracker.state().velocity[i], twist[i], 1e-9);
  }
  EXPECT_FALSE(tracker.track_velocity(twist, CartesianArray{}));
}

TEST(CartesianVelocityTracker, StopsAboveMinHeight) {
  auto tracker = make_tracker<CartesianVelocityTracker>(ready_ee_pose());
  const CartesianArray down{0.0, 0.0, -0.3, 0.0, 0.0, 0.0, 0.0};
  std::vector<CartesianState> commands{tracker.state(), tracker.state()};
  for (int k = 0; k < 5000; ++k) {
    tracker.track_velocity(down, CartesianArray{});
    commands.push_back(tracker.state());
  }
  const double min_height = tracker.settings().min_height;
  for (const auto &command : commands) {
    ASSERT_GE(command.pose.p[2], min_height - 1e-9);
  }
  EXPECT_NEAR(tracker.state().velocity[2], 0.0, 1e-4);
  expect_cartesian_within_limits(commands, tracker.settings().limits, false);
}
