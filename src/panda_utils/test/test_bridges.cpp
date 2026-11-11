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
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "panda_utils/bridge/cartesian_bridge.hpp"
#include "panda_utils/bridge/joint_bridge.hpp"
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
  void start(const std::string &mode) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"fake_robot", true}, {"publish_freq", 200.0}, {"mode", mode}});
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
        });
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
    executor->add_node(bridge->get_node_base_interface());
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
    bridge.reset();
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
  rclcpp::Subscription<uclv_panda_interfaces::msg::BridgeStatus>::SharedPtr status_sub;
  rclcpp::Subscription<uclv_panda_interfaces::msg::PandaRobotState>::SharedPtr robot_state_sub;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  std::mutex mutex;
  uclv_panda_interfaces::msg::BridgeStatus status;
  std::array<double, 7> q{};
  std::array<double, 16> ee{};
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
