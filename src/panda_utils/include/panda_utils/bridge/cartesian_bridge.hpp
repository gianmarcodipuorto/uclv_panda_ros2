#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <franka/control_types.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_panda_interfaces/action/cart_traj.hpp"
#include "uclv_panda_interfaces/srv/run_preset.hpp"

#include "panda_utils/bridge/bridge_base.hpp"
#include "panda_utils/bridge/bridge_common.hpp"
#include "panda_utils/bridge/pipeline.hpp"
#include "panda_utils/bridge/playback_clock.hpp"

namespace panda_bridge {

enum class CartesianInterface : uint8_t { kPose, kVelocities };

// What changes between the Cartesian bridges.
template <CartesianInterface I>
struct CartesianInterfaceTraits;

template <>
struct CartesianInterfaceTraits<CartesianInterface::kPose> {
  using Pipeline = CartesianPosePipeline;
  // Pose of the end effector (O_T_EE) in the base frame
  using CommandMsg = geometry_msgs::msg::PoseStamped;
  using FrankaCommand = franka::CartesianPose;
  static constexpr const char *kNodeName = "panda_cartesian_position_bridge";
  static constexpr const char *kCommandTopic = "/panda/cartesian_position_command";
  static constexpr const char *kQuantity = "end effector poses";
};

template <>
struct CartesianInterfaceTraits<CartesianInterface::kVelocities> {
  using Pipeline = CartesianVelocityPipeline;
  // Twist of the end effector in the base frame
  using CommandMsg = geometry_msgs::msg::TwistStamped;
  using FrankaCommand = franka::CartesianVelocities;
  static constexpr const char *kNodeName = "panda_cartesian_velocity_bridge";
  static constexpr const char *kCommandTopic = "/panda/cartesian_velocity_command";
  static constexpr const char *kQuantity = "end effector twists";
};

// Cartesian bridge, one per libfranka Cartesian command: end effector pose or twist.
//   mode external: follows the command topic; the samples are played back by header.stamp
//   mode internal: executes /panda/cart_traj goals and /panda/run_preset presets, generated
//                  inside the control loop; /panda/stop_motion stops them smoothly
// The commanded end effector stays above critical_height. With use_elbow_control the elbow
// (joint 3) is commanded too: held during streams, moved by the goals with desired_elbow.
template <CartesianInterface I>
class CartesianBridge : public BridgeBase {
 public:
  using Traits = CartesianInterfaceTraits<I>;
  using CartTraj = uclv_panda_interfaces::action::CartTraj;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CartTraj>;
  using CommandMsg = typename Traits::CommandMsg;
  using FrankaCommand = typename Traits::FrankaCommand;

  explicit CartesianBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~CartesianBridge() override;

 protected:
  bool configure_bridge() override;
  void cleanup_bridge() override;
  bool prepare_control(std::string &error) override;
  void on_control_started() override;
  void run_control(RobotBackend &robot) override;
  void request_stop() override { stop_requested_.store(true, std::memory_order_release); }
  std::string finish_reason() const override;
  void fill_status(BridgeStatus &status) override;
  void process_rt_events() override;

 private:
  using Pipeline = typename Traits::Pipeline;
  using Snapshot = PipelineSnapshot<CartesianState>;

  FrankaCommand rt_update(const franka::RobotState &state, franka::Duration period);
  CartesianFeedback rt_feedback(const franka::RobotState &state) const;
  FrankaCommand franka_command(const CartesianState &command) const;
  FrankaCommand hold_command(const franka::RobotState &state) const;
  void rt_publish_snapshot();

  bool read_bridge_parameters(PipelineTypes::Config &config, std::string &error);
  void on_command(const typename CommandMsg::SharedPtr msg);
  static bool command_values(const CommandMsg &msg, typename Pipeline::Values &values);
  bool push_request(const CartesianMotionRequest &request);
  bool start_preset(const std::string &preset, double duration, double amplitude, std::string &message);
  Snapshot snapshot();
  bool accepts_motions() const { return internal_mode_.load() && control_running() && !control_failed(); }

  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const CartTraj::Goal> goal);
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> handle);
  void on_accepted(const std::shared_ptr<GoalHandle> handle);
  void update_goals();

  // Configuration of the current control run (written in prepare_control)
  PipelineTypes::Config config_{};
  std::string auto_preset_;
  franka::ControllerMode controller_mode_{franka::ControllerMode::kCartesianImpedance};

  // Owned by the robot thread
  Pipeline pipeline_;
  PlaybackClock playback_clock_;
  Snapshot rt_previous_{};
  // Side of joint 4 in the elbow commands: kept from the start of the run, as the robot requires
  double elbow_sign_{-1.0};
  bool non_finite_command_{false};

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> accepting_samples_{false};
  std::atomic<bool> internal_mode_{false};

  SpscQueue<typename Pipeline::Sample, 256> samples_;
  SpscQueue<CartesianMotionRequest, 16> requests_;
  std::mutex requests_mutex_;  // requests come from several executor threads
  SpscQueue<RtEvent, 256> events_;
  SnapshotBox<Snapshot> snapshot_box_;

  typename rclcpp::Subscription<CommandMsg>::SharedPtr command_sub_;
  typename rclcpp_action::Server<CartTraj>::SharedPtr action_server_;
  rclcpp::Service<uclv_panda_interfaces::srv::RunPreset>::SharedPtr preset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr goal_timer_;

  GoalTracker<CartTraj> goals_;
  std::atomic<uint64_t> next_motion_id_{1};

  // Command stream, from the subscription to the publisher thread
  StreamStatistics stream_stats_;
  std::atomic<uint64_t> rejected_samples_{0};
  std::atomic<uint64_t> dropped_samples_{0};
  int64_t last_rejection_log_ns_{0};
};

using CartesianPoseBridge = CartesianBridge<CartesianInterface::kPose>;
using CartesianVelocitiesBridge = CartesianBridge<CartesianInterface::kVelocities>;

// Instantiated in cartesian_bridge.cpp
extern template class CartesianBridge<CartesianInterface::kPose>;
extern template class CartesianBridge<CartesianInterface::kVelocities>;

}  // namespace panda_bridge
