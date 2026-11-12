#include <gtest/gtest.h>

#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "panda_utils/bridge/gripper_backend.hpp"
#include "panda_utils/bridge/gripper_server.hpp"

using panda_bridge::FakeGripperBackend;
using panda_bridge::GripperServer;
using Gripper = uclv_panda_interfaces::action::Gripper;
using namespace std::chrono_literals;

namespace {

constexpr double kMaxWidth = 0.08;

// The fake hand reproduces the fingers seen sticking on a real one: resting on the outer end
// stop, a grasp does not start them and reports a grasp where they are
TEST(FakeGripper, GraspDoesNotStartFingersStuckOnTheOuterEndStop) {
  FakeGripperBackend hand(0.0, kMaxWidth, FakeGripperBackend::Jam::kAtOuterEndStop);
  EXPECT_TRUE(hand.grasp(0.0, 0.1, 20.0, 0.005, 1.0));
  EXPECT_NEAR(hand.read_once().width, kMaxWidth, 1e-9);
  // A position move starts them
  EXPECT_TRUE(hand.move(kMaxWidth - 0.002, 0.1));
  EXPECT_TRUE(hand.grasp(0.0, 0.1, 20.0, 0.005, 1.0));
  EXPECT_LT(hand.read_once().width, 1e-6);
}

class GripperServerTest : public ::testing::Test {
 protected:
  // Never on the domain of the user, as test_bridges: a bridge connected to the real robot may be
  // running there
  static void SetUpTestSuite() {
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
    rclcpp::InitOptions options;
    options.set_domain_id(static_cast<size_t>(215 + getpid() % 18));
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void start(FakeGripperBackend::Jam jam, double object_width = 0.0) {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("gripper_server_test");
    auto backend = std::make_unique<FakeGripperBackend>(object_width, kMaxWidth, jam);
    hand = backend.get();
    GripperServer::Params params;
    params.joint_names = {"fer_finger_joint1", "fer_finger_joint2"};
    server = std::make_unique<GripperServer>(
        *node, std::move(backend), params, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    client = rclcpp::Node::make_shared("gripper_server_test_client");
    action_client = rclcpp_action::create_client<Gripper>(client, "/panda/gripper");
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3);
    executor->add_node(node->get_node_base_interface());
    executor->add_node(client);
    spinner = std::thread([this] { executor->spin(); });
  }

  void TearDown() override {
    if (executor) {
      executor->cancel();
    }
    if (spinner.joinable()) {
      spinner.join();
    }
    action_client.reset();
    server.reset();
    client.reset();
    node.reset();
  }

  struct Outcome {
    rclcpp_action::ResultCode code{rclcpp_action::ResultCode::UNKNOWN};
    Gripper::Result result;
  };

  Outcome run(uint8_t command, double width = 0.0) {
    Outcome outcome;
    if (!action_client->wait_for_action_server(2s)) {
      return outcome;
    }
    Gripper::Goal goal;
    goal.command = command;
    goal.width = width;
    auto handle = action_client->async_send_goal(goal);
    if (handle.wait_for(2s) != std::future_status::ready || !handle.get()) {
      return outcome;
    }
    auto result = action_client->async_get_result(handle.get());
    if (result.wait_for(5s) != std::future_status::ready) {
      return outcome;
    }
    const auto wrapped = result.get();
    outcome.code = wrapped.code;
    outcome.result = *wrapped.result;
    return outcome;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  FakeGripperBackend *hand{nullptr};
  std::unique_ptr<GripperServer> server;
  rclcpp::Node::SharedPtr client;
  rclcpp_action::Client<Gripper>::SharedPtr action_client;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
};

}  // namespace

TEST_F(GripperServerTest, CloseFreesFingersStuckOnTheOuterEndStop) {
  start(FakeGripperBackend::Jam::kAtOuterEndStop);
  Outcome outcome = run(Gripper::Goal::CLOSE);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_LT(outcome.result.width, 1e-3);
  EXPECT_EQ(outcome.result.message, "closed: nothing between the fingers");
  // Open goes back onto the end stop: the next close frees the fingers again
  ASSERT_EQ(run(Gripper::Goal::OPEN).code, rclcpp_action::ResultCode::SUCCEEDED);
  outcome = run(Gripper::Goal::CLOSE);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_LT(outcome.result.width, 1e-3);
}

TEST_F(GripperServerTest, FingersThatDoNotMoveFail) {
  start(FakeGripperBackend::Jam::kAlways);
  for (const auto &[command, width] : {std::pair<uint8_t, double>{Gripper::Goal::CLOSE, 0.0},
                                       std::pair<uint8_t, double>{Gripper::Goal::CLOSE, 0.03},
                                       std::pair<uint8_t, double>{Gripper::Goal::MOVE, 0.04}}) {
    const Outcome outcome = run(command, width);
    EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED) << int(command) << " " << width;
    EXPECT_FALSE(outcome.result.success);
    EXPECT_EQ(outcome.result.message, "the fingers did not move: try again, or run a homing (command 3)");
    EXPECT_NEAR(outcome.result.width, kMaxWidth, 1e-9);
  }
  // Already open: nothing had to move
  EXPECT_EQ(run(Gripper::Goal::OPEN).code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(GripperServerTest, CloseOnNothingStopsSqueezing) {
  start(FakeGripperBackend::Jam::kNone);
  const Outcome outcome = run(Gripper::Goal::CLOSE);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_LT(outcome.result.width, 1e-3);
  EXPECT_FALSE(hand->grasping());
}

TEST_F(GripperServerTest, CloseOnAnObjectKeepsHoldingIt) {
  start(FakeGripperBackend::Jam::kNone, 0.03);
  const Outcome outcome = run(Gripper::Goal::CLOSE);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(outcome.result.width, 0.03, 1e-3);
  EXPECT_TRUE(hand->grasping());
}
