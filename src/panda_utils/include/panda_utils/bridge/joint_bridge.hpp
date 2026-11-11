#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <franka/control_types.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uclv_panda_interfaces/action/joint_traj.hpp"
#include "uclv_panda_interfaces/msg/joints_commanded_positions.hpp"
#include "uclv_panda_interfaces/srv/run_preset.hpp"

#include "panda_utils/bridge/bridge_base.hpp"
#include "panda_utils/bridge/bridge_common.hpp"
#include "panda_utils/bridge/pipeline.hpp"
#include "panda_utils/bridge/playback_clock.hpp"

namespace panda_bridge {

enum class JointInterface : uint8_t { kPositions, kVelocities, kTorques };

// What changes between the joint bridges.
template <JointInterface I>
struct JointInterfaceTraits;

template <>
struct JointInterfaceTraits<JointInterface::kPositions> {
  using Pipeline = JointPositionPipeline;
  using CommandMsg = uclv_panda_interfaces::msg::JointsCommandedPositions;
  using FrankaCommand = franka::JointPositions;
  static constexpr const char *kNodeName = "panda_joint_positions_bridge";
  static constexpr const char *kCommandTopic = "/panda/joint_position_command";
  static constexpr const char *kQuantity = "joint positions";
};

template <>
struct JointInterfaceTraits<JointInterface::kVelocities> {
  using Pipeline = JointVelocityPipeline;
  // The velocity field, in the order of joint_names
  using CommandMsg = sensor_msgs::msg::JointState;
  using FrankaCommand = franka::JointVelocities;
  static constexpr const char *kNodeName = "panda_joint_velocities_bridge";
  static constexpr const char *kCommandTopic = "/panda/joint_velocity_command";
  static constexpr const char *kQuantity = "joint velocities";
};

template <>
struct JointInterfaceTraits<JointInterface::kTorques> {
  using Pipeline = JointTorquePipeline;
  // The effort field, in the order of joint_names, without gravity: the robot compensates it
  using CommandMsg = sensor_msgs::msg::JointState;
  using FrankaCommand = franka::Torques;
  static constexpr const char *kNodeName = "panda_joint_torques_bridge";
  static constexpr const char *kCommandTopic = "/panda/joint_torque_command";
  static constexpr const char *kQuantity = "joint torques";
};

// Joint bridge, one per libfranka joint command: positions, velocities or torques.
//   mode external: follows the command topic, like a forward command controller of ros2_control;
//                  the samples are played back by header.stamp
//   mode internal: executes /panda/joint_traj goals and /panda/run_preset presets, generated
//                  inside the control loop; /panda/stop_motion stops them smoothly. The torque
//                  bridge follows them with a joint impedance law.
template <JointInterface I>
class JointBridge : public BridgeBase {
 public:
  using Traits = JointInterfaceTraits<I>;
  using JointTraj = uclv_panda_interfaces::action::JointTraj;
  using GoalHandle = rclcpp_action::ServerGoalHandle<JointTraj>;
  using CommandMsg = typename Traits::CommandMsg;
  using FrankaCommand = typename Traits::FrankaCommand;

  explicit JointBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~JointBridge() override;

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
  using Snapshot = PipelineSnapshot<JointArray>;

  FrankaCommand rt_update(const franka::RobotState &state, franka::Duration period);
  JointFeedback rt_feedback(const franka::RobotState &state) const;
  void rt_publish_snapshot();

  bool read_bridge_parameters(PipelineTypes::Config &config, std::string &error);
  bool read_torque_parameters(TorqueSettings &settings, std::string &error);
  void on_command(const typename CommandMsg::SharedPtr msg);
  static bool command_values(const CommandMsg &msg, JointArray &values);
  bool push_request(const JointMotionRequest &request);
  bool start_preset(const std::string &preset, double duration, double amplitude, std::string &message);
  Snapshot snapshot();
  bool accepts_motions() const { return internal_mode_.load() && control_running() && !control_failed(); }

  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const JointTraj::Goal> goal);
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> handle);
  void on_accepted(const std::shared_ptr<GoalHandle> handle);
  void update_goals();

  // Configuration of the current control run (written in prepare_control)
  PipelineTypes::Config config_{};
  std::string auto_preset_;
  franka::ControllerMode controller_mode_{franka::ControllerMode::kJointImpedance};

  // Owned by the robot thread
  Pipeline pipeline_;
  PlaybackClock playback_clock_;
  Snapshot rt_previous_{};
  const franka::Model *rt_model_{nullptr};
  bool non_finite_command_{false};

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> accepting_samples_{false};
  std::atomic<bool> internal_mode_{false};

  SpscQueue<typename Pipeline::Sample, 256> samples_;
  SpscQueue<JointMotionRequest, 16> requests_;
  std::mutex requests_mutex_;  // requests come from several executor threads
  SpscQueue<RtEvent, 256> events_;
  SnapshotBox<Snapshot> snapshot_box_;

  typename rclcpp::Subscription<CommandMsg>::SharedPtr command_sub_;
  typename rclcpp_action::Server<JointTraj>::SharedPtr action_server_;
  rclcpp::Service<uclv_panda_interfaces::srv::RunPreset>::SharedPtr preset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr goal_timer_;

  GoalTracker<JointTraj> goals_;
  std::atomic<uint64_t> next_motion_id_{1};

  // Command stream, from the subscription to the publisher thread
  StreamStatistics stream_stats_;
  std::atomic<uint64_t> rejected_samples_{0};
  std::atomic<uint64_t> dropped_samples_{0};
  int64_t last_rejection_log_ns_{0};
};

using JointPositionsBridge = JointBridge<JointInterface::kPositions>;
using JointVelocitiesBridge = JointBridge<JointInterface::kVelocities>;
using JointTorquesBridge = JointBridge<JointInterface::kTorques>;

// Instantiated in joint_bridge.cpp
extern template class JointBridge<JointInterface::kPositions>;
extern template class JointBridge<JointInterface::kVelocities>;
extern template class JointBridge<JointInterface::kTorques>;

}  // namespace panda_bridge
