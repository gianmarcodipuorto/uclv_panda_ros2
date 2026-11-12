#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <string>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "panda_utils/bridge/cartesian_bridge.hpp"
#include "panda_utils/bridge/joint_bridge.hpp"
#include "uclv_panda_interfaces/action/gripper.hpp"
#include "uclv_panda_interfaces/msg/bridge_status.hpp"

using lifecycle_msgs::msg::State;
using panda_bridge::CartesianPoseBridge;
using panda_bridge::CartesianVelocitiesBridge;
using panda_bridge::JointPositionsBridge;
using panda_bridge::JointTorquesBridge;
using panda_bridge::JointVelocitiesBridge;
using namespace std::chrono_literals;

namespace {

// Initial configuration of the fake robot backend
const std::array<double, 7> kStart{0.0, -M_PI_4, 0.0, -3 * M_PI_4, 0.0, M_PI_2, M_PI_4};

using JointTraj = uclv_panda_interfaces::action::JointTraj;
using CartTraj = uclv_panda_interfaces::action::CartTraj;
using Gripper = uclv_panda_interfaces::action::Gripper;
using GripperGoalHandle = rclcpp_action::ClientGoalHandle<Gripper>;

// Opening of the fake gripper when open [m]
constexpr double kGripperMaxWidth = 0.08;

struct GripperOutcome {
  rclcpp_action::ResultCode code{rclcpp_action::ResultCode::UNKNOWN};
  Gripper::Result result;
};

// Initial end effector pose of the fake robot: identity orientation at (0.3, 0, 0.5)
const std::array<double, 3> kStartEE{0.3, 0.0, 0.5};

class BridgeTest : public ::testing::Test {
 protected:
  // Never on the domain of the user: a bridge connected to the real robot may be running there
  // and would receive the commands of the test. ROS_DOMAIN_ID from the environment is ignored
  // (colcon's isolated runner would keep it too); domains 215-232 are outside the range that
  // domain_coordinator assigns, and discovery stays on this machine.
  static void SetUpTestSuite() {
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
    rclcpp::InitOptions options;
    options.set_domain_id(static_cast<size_t>(215 + getpid() % 18));
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  template <typename Bridge>
  void start(const std::string &mode, const std::vector<rclcpp::Parameter> &extra_parameters = {}) {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters{{"fake_robot", true}, {"publish_freq", 200.0}, {"mode", mode}};
    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());
    options.parameter_overrides(parameters);
    bridge = std::make_shared<Bridge>(options);
    client = rclcpp::Node::make_shared("bridge_test_client");
    status_sub = client->create_subscription<uclv_panda_interfaces::msg::BridgeStatus>(
        "/panda/bridge_status", 10, [this](uclv_panda_interfaces::msg::BridgeStatus::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex);
          status = *msg;
        });
    robot_state_sub = client->create_subscription<uclv_panda_interfaces::msg::PandaRobotState>(
        "/panda/robot_state", 10, [this](uclv_panda_interfaces::msg::PandaRobotState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex);
          q = msg->q;
          ee = msg->o_t_ee;
          has_model = msg->has_model;
        });
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 6);
    executor->add_node(bridge->get_node_base_interface());
    executor->add_node(client);
    spinner = std::thread([this] { executor->spin(); });
  }

  void TearDown() override { stop_all(); }

  // Stops spinning and destroys the bridge and the client, so that another bridge can start
  void stop_all() {
    if (executor) {
      executor->cancel();
    }
    if (spinner.joinable()) {
      spinner.join();
    }
    executor.reset();
    gripper_client.reset();
    status_sub.reset();
    robot_state_sub.reset();
    client.reset();
    bridge.reset();
  }

  // Sends a gripper goal: its handle, or nullptr if the goal was rejected or there is no server
  GripperGoalHandle::SharedPtr send_gripper_goal(uint8_t command, double width = 0.0, double speed = 0.0,
                                                 double force = 0.0) {
    if (!gripper_client) {
      gripper_client = rclcpp_action::create_client<Gripper>(client, "/panda/gripper");
    }
    if (!gripper_client->wait_for_action_server(2s)) {
      return nullptr;
    }
    Gripper::Goal goal;
    goal.command = command;
    goal.width = width;
    goal.speed = speed;
    goal.force = force;
    auto future = gripper_client->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  GripperOutcome wait_gripper_result(const GripperGoalHandle::SharedPtr &handle) {
    GripperOutcome outcome;
    if (!handle) {
      return outcome;
    }
    auto future = gripper_client->async_get_result(handle);
    if (future.wait_for(5s) != std::future_status::ready) {
      return outcome;
    }
    const auto wrapped = future.get();
    outcome.code = wrapped.code;
    outcome.result = *wrapped.result;
    return outcome;
  }

  GripperOutcome run_gripper(uint8_t command, double width = 0.0, double speed = 0.0, double force = 0.0) {
    return wait_gripper_result(send_gripper_goal(command, width, speed, force));
  }

  // The gripper opens and moves while the bridge is inactive and while it is active
  template <typename Bridge>
  void expect_gripper_served(const std::string &mode) {
    start<Bridge>(mode);
    ASSERT_EQ(bridge->configure().id(), State::PRIMARY_STATE_INACTIVE);
    GripperOutcome outcome = run_gripper(Gripper::Goal::MOVE, 0.07);
    EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED) << bridge->get_name() << " inactive";
    EXPECT_NEAR(outcome.result.width, 0.07, 1e-3);
    ASSERT_EQ(bridge->activate().id(), State::PRIMARY_STATE_ACTIVE);
    outcome = run_gripper(Gripper::Goal::OPEN);
    EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED) << bridge->get_name() << " active, " << mode;
    EXPECT_NEAR(outcome.result.width, kGripperMaxWidth, 1e-3);
    shut_down();
    stop_all();
  }

  std::string state() {
    std::lock_guard<std::mutex> lock(mutex);
    return status.state;
  }

  std::array<double, 7> joints() {
    std::lock_guard<std::mutex> lock(mutex);
    return q;
  }

  // Measured end effector position
  std::array<double, 3> ee_position() {
    std::lock_guard<std::mutex> lock(mutex);
    return {ee[12], ee[13], ee[14]};
  }

  bool wait_for_state(const std::string &expected, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (state() == expected) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void cycle_lifecycle(int cycles) {
    ASSERT_EQ(bridge->configure().id(), State::PRIMARY_STATE_INACTIVE);
    for (int i = 0; i < cycles; ++i) {
      const auto begin = std::chrono::steady_clock::now();
      ASSERT_EQ(bridge->activate().id(), State::PRIMARY_STATE_ACTIVE) << "cycle " << i;
      ASSERT_EQ(bridge->deactivate().id(), State::PRIMARY_STATE_INACTIVE) << "cycle " << i;
      EXPECT_LT(std::chrono::steady_clock::now() - begin, 1s) << "cycle " << i;
    }
    ASSERT_EQ(bridge->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  }

  void activate() {
    ASSERT_EQ(bridge->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(bridge->activate().id(), State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(wait_for_state("holding", 2s));
  }

  void shut_down() {
    ASSERT_EQ(bridge->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(bridge->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  }

  // Publishes make(t) at 1 kHz for the given time, stamped at sampling time
  template <typename Msg>
  void stream(const std::string &topic, std::chrono::milliseconds duration,
              const std::function<void(double, Msg &)> &make) {
    auto publisher = client->create_publisher<Msg>(topic, 10);
    const auto t0 = client->now();
    auto next = std::chrono::steady_clock::now();
    const auto end = next + duration;
    while (next < end) {
      Msg msg;
      msg.header.stamp = client->now();
      make((rclcpp::Time(msg.header.stamp) - t0).seconds(), msg);
      publisher->publish(msg);
      next += 1ms;
      std::this_thread::sleep_until(next);
    }
  }

  // Sends a CartTraj goal to a pose with the initial orientation and waits for the result
  rclcpp_action::ResultCode run_cart_goal(const std::array<double, 3> &position, double total_time) {
    auto action_client = rclcpp_action::create_client<CartTraj>(client, "/panda/cart_traj");
    if (!action_client->wait_for_action_server(2s)) {
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    CartTraj::Goal goal;
    goal.desired_pose.position.x = position[0];
    goal.desired_pose.position.y = position[1];
    goal.desired_pose.position.z = position[2];
    goal.desired_pose.orientation.w = 1.0;
    goal.total_time = total_time;
    goal.desired_elbow = std::nan("");
    auto goal_handle = action_client->async_send_goal(goal).get();
    if (!goal_handle) {
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    auto result = action_client->async_get_result(goal_handle);
    if (result.wait_for(std::chrono::duration<double>(total_time + 3.0)) != std::future_status::ready) {
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    return result.get().code;
  }

  // Sends a JointTraj goal and waits for the result
  rclcpp_action::ResultCode run_goal(const std::array<double, 7> &goal_position, double total_time) {
    auto action_client = rclcpp_action::create_client<JointTraj>(client, "/panda/joint_traj");
    if (!action_client->wait_for_action_server(2s)) {
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    JointTraj::Goal goal;
    goal.desired_joint_cmd.position.assign(goal_position.begin(), goal_position.end());
    goal.total_time = total_time;
    auto goal_handle = action_client->async_send_goal(goal).get();
    if (!goal_handle) {
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    auto result = action_client->async_get_result(goal_handle);
    if (result.wait_for(std::chrono::duration<double>(total_time + 3.0)) != std::future_status::ready) {
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    return result.get().code;
  }

  bool run_preset(const std::string &name, double duration) {
    auto preset = client->create_client<uclv_panda_interfaces::srv::RunPreset>("/panda/run_preset");
    if (!preset->wait_for_service(2s)) {
      return false;
    }
    auto request = std::make_shared<uclv_panda_interfaces::srv::RunPreset::Request>();
    request->preset = name;
    request->duration = duration;
    auto response = preset->async_send_request(request);
    return response.wait_for(2s) == std::future_status::ready && response.get()->accepted;
  }

  std::shared_ptr<panda_bridge::BridgeBase> bridge;
  rclcpp::Node::SharedPtr client;
  rclcpp_action::Client<Gripper>::SharedPtr gripper_client;
  rclcpp::Subscription<uclv_panda_interfaces::msg::BridgeStatus>::SharedPtr status_sub;
  rclcpp::Subscription<uclv_panda_interfaces::msg::PandaRobotState>::SharedPtr robot_state_sub;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  std::mutex mutex;
  uclv_panda_interfaces::msg::BridgeStatus status;
  std::array<double, 7> q{};
  std::array<double, 16> ee{};
  std::optional<bool> has_model;
};

}  // namespace

TEST_F(BridgeTest, PositionsActivateDeactivateNeverHangs) {
  start<JointPositionsBridge>("external");
  cycle_lifecycle(20);
}

TEST_F(BridgeTest, PositionsFollowStreamAndStopWhenItStalls) {
  start<JointPositionsBridge>("external");
  activate();
  // Ramp on joint 1 at 0.2 rad/s for one second
  stream<JointPositionsBridge::CommandMsg>("/panda/joint_position_command", 1000ms, [](double t, auto &msg) {
    msg.positions = kStart;
    msg.positions[0] += 0.2 * t;
  });
  EXPECT_EQ(state(), "moving");
  // The client stopped while the robot was moving: smooth stop, then hold
  ASSERT_TRUE(wait_for_state("holding", 2s));
  EXPECT_NEAR(joints()[0], 0.2, 0.01);
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(status.stalls, 1u);
    EXPECT_EQ(status.mode, "external");
    // The simulated robot has no dynamic model
    ASSERT_TRUE(has_model.has_value());
    EXPECT_FALSE(*has_model);
  }
  shut_down();
}

TEST_F(BridgeTest, PositionsExecuteGoalsAndPresetsInInternalMode) {
  start<JointPositionsBridge>("internal");
  activate();
  auto goal = kStart;
  goal[3] += 0.1;
  EXPECT_EQ(run_goal(goal, 0.5), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_for_state("holding", 1s));
  EXPECT_NEAR(joints()[3], kStart[3] + 0.1, 1e-6);

  EXPECT_TRUE(run_preset("quintic_joint1", 0.5));
  ASSERT_TRUE(wait_for_state("moving", 1s));
  ASSERT_TRUE(wait_for_state("holding", 3s));
  EXPECT_NEAR(joints()[0], kStart[0] + 0.2, 1e-6);
  shut_down();
}

TEST_F(BridgeTest, VelocitiesActivateDeactivateNeverHangs) {
  start<JointVelocitiesBridge>("external");
  cycle_lifecycle(5);
}

TEST_F(BridgeTest, VelocitiesFollowStreamAndStopWhenItStalls) {
  start<JointVelocitiesBridge>("external");
  activate();
  // Joint 1 at 0.2 rad/s for one second, ramped up in 0.2 s
  stream<JointVelocitiesBridge::CommandMsg>("/panda/joint_velocity_command", 1000ms, [](double t, auto &msg) {
    msg.velocity.assign(7, 0.0);
    msg.velocity[0] = 0.2 * std::min(1.0, t / 0.2);
  });
  EXPECT_EQ(state(), "moving");
  ASSERT_TRUE(wait_for_state("holding", 2s));
  // 0.18 rad streamed, plus the gap bridged before the stall and the deceleration
  EXPECT_NEAR(joints()[0], 0.185, 0.01);
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(status.stalls, 1u);
  }
  shut_down();
}

TEST_F(BridgeTest, VelocitiesExecuteGoalsInInternalMode) {
  start<JointVelocitiesBridge>("internal");
  activate();
  auto goal = kStart;
  goal[3] += 0.1;
  EXPECT_EQ(run_goal(goal, 0.5), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_for_state("holding", 1s));
  EXPECT_NEAR(joints()[3], kStart[3] + 0.1, 1e-5);
  shut_down();
}

TEST_F(BridgeTest, TorquesActivateDeactivateNeverHangs) {
  start<JointTorquesBridge>("external");
  cycle_lifecycle(5);
}

TEST_F(BridgeTest, TorquesFollowStreamThenHold) {
  start<JointTorquesBridge>("external");
  activate();
  const auto before = joints();
  // A small torque on joint 7 for half a second: the joint accelerates
  stream<JointTorquesBridge::CommandMsg>("/panda/joint_torque_command", 500ms, [](double, auto &msg) {
    msg.effort.assign(7, 0.0);
    msg.effort[6] = 0.05;
  });
  EXPECT_EQ(state(), "moving");
  // Stream over: damping, then hold where the joint stopped
  ASSERT_TRUE(wait_for_state("holding", 2s));
  EXPECT_GT(joints()[6] - before[6], 0.05);
  shut_down();
}

TEST_F(BridgeTest, TorquesExecuteGoalsAndGravityCompensationInInternalMode) {
  start<JointTorquesBridge>("internal");
  activate();
  auto goal = kStart;
  goal[3] += 0.1;
  EXPECT_EQ(run_goal(goal, 1.0), rclcpp_action::ResultCode::SUCCEEDED);
  // The hold keeps the goal
  std::this_thread::sleep_for(500ms);
  EXPECT_NEAR(joints()[3], kStart[3] + 0.1, 2e-3);

  EXPECT_TRUE(run_preset("gravity_compensation", 0.0));
  ASSERT_TRUE(wait_for_state("moving", 1s));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(status.motion, "gravity_compensation");
  }
  auto stop = client->create_client<std_srvs::srv::Trigger>("/panda/stop_motion");
  ASSERT_TRUE(stop->wait_for_service(2s));
  auto response = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(response.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(response.get()->success);
  ASSERT_TRUE(wait_for_state("holding", 2s));
  shut_down();
}

TEST_F(BridgeTest, CartesianPoseActivateDeactivateNeverHangs) {
  start<CartesianPoseBridge>("external");
  cycle_lifecycle(5);
}

TEST_F(BridgeTest, CartesianPoseFollowsStreamAndStopsWhenItStalls) {
  start<CartesianPoseBridge>("external");
  activate();
  // Along x at 5 cm/s for one second, with the initial orientation
  stream<CartesianPoseBridge::CommandMsg>("/panda/cartesian_position_command", 1000ms, [](double t, auto &msg) {
    msg.pose.position.x = kStartEE[0] + 0.05 * t;
    msg.pose.position.y = kStartEE[1];
    msg.pose.position.z = kStartEE[2];
    msg.pose.orientation.w = 1.0;
  });
  EXPECT_EQ(state(), "moving");
  ASSERT_TRUE(wait_for_state("holding", 2s));
  EXPECT_NEAR(ee_position()[0], kStartEE[0] + 0.05, 0.005);
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(status.stalls, 1u);
  }
  shut_down();
}

TEST_F(BridgeTest, CartesianPoseExecutesGoalsAndPresetsInInternalMode) {
  start<CartesianPoseBridge>("internal");
  activate();
  const std::array<double, 3> goal{0.35, 0.05, 0.45};
  EXPECT_EQ(run_cart_goal(goal, 1.0), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_for_state("holding", 1s));
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(ee_position()[i], goal[i], 1e-6);
  }
  // Out along x and back to where the robot is
  EXPECT_TRUE(run_preset("line_x", 1.0));
  ASSERT_TRUE(wait_for_state("moving", 1s));
  ASSERT_TRUE(wait_for_state("holding", 3s));
  EXPECT_NEAR(ee_position()[0], goal[0], 1e-6);
  shut_down();
}

TEST_F(BridgeTest, CartesianVelocityFollowsTwistStream) {
  start<CartesianVelocitiesBridge>("external");
  activate();
  // Along x at 5 cm/s for one second, ramped up in 0.2 s
  stream<CartesianVelocitiesBridge::CommandMsg>("/panda/cartesian_velocity_command", 1000ms, [](double t, auto &msg) {
    msg.twist.linear.x = 0.05 * std::min(1.0, t / 0.2);
  });
  EXPECT_EQ(state(), "moving");
  ASSERT_TRUE(wait_for_state("holding", 2s));
  // 4.5 cm streamed, plus the gap bridged before the stall and the deceleration
  EXPECT_NEAR(ee_position()[0], kStartEE[0] + 0.046, 0.003);
  shut_down();
}

TEST_F(BridgeTest, GripperServedByEveryBridgeInEveryMode) {
  expect_gripper_served<JointPositionsBridge>("external");
  expect_gripper_served<JointVelocitiesBridge>("internal");
  expect_gripper_served<JointTorquesBridge>("external");
  expect_gripper_served<JointTorquesBridge>("internal");
  expect_gripper_served<CartesianPoseBridge>("internal");
  expect_gripper_served<CartesianVelocitiesBridge>("external");
}

TEST_F(BridgeTest, GripperClosesOpensAndPublishesItsState) {
  start<JointPositionsBridge>("external");
  std::mutex state_mutex;
  sensor_msgs::msg::JointState gripper_state;
  auto state_sub = client->create_subscription<sensor_msgs::msg::JointState>(
      "/panda/gripper_state", 10, [&](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        gripper_state = *msg;
      });
  activate();

  // Nothing between the fingers: they close fully
  GripperOutcome outcome = run_gripper(Gripper::Goal::CLOSE);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(outcome.result.success);
  EXPECT_LT(outcome.result.width, 1e-3);
  EXPECT_EQ(outcome.result.message, "closed: nothing between the fingers");
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    ASSERT_EQ(gripper_state.name.size(), 2u);
    EXPECT_EQ(gripper_state.name[0], "fer_finger_joint1");
    EXPECT_LT(gripper_state.position[0], 1e-3);
  }

  outcome = run_gripper(Gripper::Goal::OPEN);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(outcome.result.width, kGripperMaxWidth, 1e-3);
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    EXPECT_NEAR(gripper_state.position[0] + gripper_state.position[1], kGripperMaxWidth, 1e-3);
  }
  // The arm never noticed
  EXPECT_EQ(state(), "holding");
  state_sub.reset();
  shut_down();
}

TEST_F(BridgeTest, GripperGraspsAnObject) {
  start<CartesianPoseBridge>("internal", {{"fake_gripper_object_width", 0.03}});
  activate();
  GripperOutcome outcome = run_gripper(Gripper::Goal::CLOSE, 0.0, 0.0, 40.0);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(outcome.result.width, 0.03, 1e-3);
  EXPECT_EQ(outcome.result.message, "holding an object 30.0 mm wide");

  // An object of the expected width is grasped, one of another width is not
  ASSERT_EQ(run_gripper(Gripper::Goal::OPEN).code, rclcpp_action::ResultCode::SUCCEEDED);
  outcome = run_gripper(Gripper::Goal::CLOSE, 0.032);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_EQ(run_gripper(Gripper::Goal::OPEN).code, rclcpp_action::ResultCode::SUCCEEDED);
  outcome = run_gripper(Gripper::Goal::CLOSE, 0.05);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(outcome.result.success);
  EXPECT_EQ(outcome.result.message, "no object of 50.0 mm: fingers at 30.0 mm");
  shut_down();
}

TEST_F(BridgeTest, GripperNewGoalPreemptsTheRunningOne) {
  start<JointVelocitiesBridge>("external");
  ASSERT_EQ(bridge->configure().id(), State::PRIMARY_STATE_INACTIVE);
  // Slow close (1.6 s), then open after 300 ms
  auto closing = send_gripper_goal(Gripper::Goal::CLOSE, 0.0, 0.05);
  ASSERT_NE(closing, nullptr);
  std::this_thread::sleep_for(300ms);
  auto opening = send_gripper_goal(Gripper::Goal::OPEN);
  ASSERT_NE(opening, nullptr);
  const GripperOutcome closed = wait_gripper_result(closing);
  EXPECT_EQ(closed.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(closed.result.message, "preempted by a newer goal");
  EXPECT_GT(closed.result.width, 0.03);
  const GripperOutcome opened = wait_gripper_result(opening);
  EXPECT_EQ(opened.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(opened.result.width, kGripperMaxWidth, 1e-3);
  ASSERT_EQ(bridge->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(BridgeTest, GripperCancelStopsTheFingers) {
  start<JointTorquesBridge>("internal");
  activate();
  auto moving = send_gripper_goal(Gripper::Goal::MOVE, 0.0, 0.05);
  ASSERT_NE(moving, nullptr);
  std::this_thread::sleep_for(300ms);
  gripper_client->async_cancel_goal(moving);
  const GripperOutcome outcome = wait_gripper_result(moving);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(outcome.result.message, "canceled");
  // Stopped on the way: 1.5 cm or so closed in 300 ms
  EXPECT_GT(outcome.result.width, 0.03);
  EXPECT_LT(outcome.result.width, kGripperMaxWidth - 0.005);
  shut_down();
}

TEST_F(BridgeTest, GripperRejectsInvalidGoals) {
  start<JointPositionsBridge>("internal");
  ASSERT_EQ(bridge->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(send_gripper_goal(Gripper::Goal::CLOSE, 0.0, 0.0, 100.0), nullptr);   // above 70 N
  EXPECT_EQ(send_gripper_goal(Gripper::Goal::MOVE, 0.2), nullptr);                // wider than the gripper
  EXPECT_EQ(send_gripper_goal(Gripper::Goal::MOVE, -0.01), nullptr);
  EXPECT_EQ(send_gripper_goal(7), nullptr);                                       // unknown command
  EXPECT_EQ(send_gripper_goal(Gripper::Goal::OPEN, 0.0, -1.0), nullptr);          // negative speed
  EXPECT_NE(send_gripper_goal(Gripper::Goal::HOMING), nullptr);
  ASSERT_EQ(bridge->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(BridgeTest, NoGripperWhenDisabled) {
  start<JointPositionsBridge>("external", {{"use_gripper", false}});
  ASSERT_EQ(bridge->configure().id(), State::PRIMARY_STATE_INACTIVE);
  auto action_client = rclcpp_action::create_client<Gripper>(client, "/panda/gripper");
  EXPECT_FALSE(action_client->wait_for_action_server(1s));
  action_client.reset();
  ASSERT_EQ(bridge->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(BridgeTest, CartesianVelocityExecutesGoalsInInternalMode) {
  start<CartesianVelocitiesBridge>("internal");
  activate();
  const std::array<double, 3> goal{0.25, -0.05, 0.55};
  EXPECT_EQ(run_cart_goal(goal, 1.0), rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_for_state("holding", 1s));
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(ee_position()[i], goal[i], 1e-5);
  }
  shut_down();
}
