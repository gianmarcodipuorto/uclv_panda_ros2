#pragma once

#include <cstdint>
#include <tuple>

#include "panda_utils/bridge/cartesian_tracker.hpp"
#include "panda_utils/bridge/jitter_buffer.hpp"
#include "panda_utils/bridge/joint_torque_tracker.hpp"
#include "panda_utils/bridge/joint_tracker.hpp"
#include "panda_utils/bridge/joint_velocity_tracker.hpp"
#include "panda_utils/bridge/pipeline_types.hpp"

namespace panda_bridge {

// Command pipeline of the bridges, independent of libfranka and ROS so that it can be tested
// offline. One step() per 1 ms control cycle produces the command of the robot. The state machine
// is the same for all the bridges; the Tracker turns references into commands the robot accepts:
//   JointTracker              joint positions      (franka::JointPositions)
//   JointVelocityTracker      joint velocities     (franka::JointVelocities)
//   JointTorqueTracker        joint torques        (franka::Torques)
//   CartesianPoseTracker      end effector pose    (franka::CartesianPose)
//   CartesianVelocityTracker  end effector twist   (franka::CartesianVelocities)
//
// External mode: timestamped samples are played back through a JitterBuffer and followed by the
// tracker. Short gaps of the stream are bridged by extrapolation, longer ones bring the robot to
// a smooth stop; the stream resumes from fresh samples. Position and pose streams should start
// from the commanded state (q_d, O_T_EE_d in /panda/robot_state), as in the libfranka examples:
// a start from the measured one is a small step, which the tracker absorbs.
// Internal mode: motions passed to request() are generated here and followed the same way.
//
// A Tracker provides the types Values (one stream sample), Feedback (robot state it needs),
// Motion and MotionPoint (internal motions), Command and Position, and the functions used below:
// configure, reset, set_feedback, align_sample, rejects_stream_start, reference_within_limits,
// follow_stream, prepare_motion, follow_motion, follow_free (if kSupportsFreeMotion), brake,
// hold, at_rest, nearly_at_rest, command, position, errors.
template <typename Tracker>
class Pipeline : public PipelineTypes {
 public:
  using Values = typename Tracker::Values;
  using Buffer = JitterBuffer<std::tuple_size<Values>::value>;
  using Sample = typename Buffer::Sample;
  using Feedback = typename Tracker::Feedback;
  using Motion = typename Tracker::Motion;
  using Request = MotionRequest<Motion>;
  using Command = typename Tracker::Command;
  using Position = typename Tracker::Position;

  void configure(const Config &config);
  // Start of a control run, from the state reported by the robot.
  void reset(const Feedback &robot, int64_t now_ns);
  // Same, for a joint pipeline with the robot at the commanded state (q = q_d).
  template <typename Array>
  void reset(const Array &q_d, const Array &dq_d, const Array &ddq_d, int64_t now_ns) {
    JointFeedback robot;
    robot.q = robot.q_d = q_d;
    robot.dq = robot.dq_d = dq_d;
    robot.ddq_d = ddq_d;
    reset(robot, now_ns);
  }
  // State of the robot, before every step(): the torque tracker is a feedback law.
  void set_feedback(const Feedback &robot) { tracker_.set_feedback(robot); }

  void push_sample(const Sample &sample);
  void request(const Request &request);
  // Smooth stop, then state kFinished. The first reason is kept.
  void request_finish(FinishReason reason);
  void step(int64_t now_ns);

  // Command of the cycle, depending on the tracker
  const Command &command() const { return tracker_.command(); }
  // Where the internal motions start from
  const Position &position() const { return tracker_.position(); }
  State state() const { return state_; }
  FinishReason finish_reason() const { return finish_reason_; }
  const Counters &counters() const { return counters_; }
  const MotionStatus &motion_status() const { return motion_status_; }
  uint64_t tracker_errors() const { return tracker_.errors(); }
  const Config &config() const { return config_; }
  const Tracker &tracker() const { return tracker_; }

 private:
  void step_holding(int64_t now_ns);
  void step_moving(int64_t now_ns);
  void step_stopping();
  typename Buffer::Reference sample(int64_t now_ns);
  bool start_stream(const typename Buffer::Reference &ref, int64_t now_ns);
  void start_motion(const Request &request);
  void follow_stream(const typename Buffer::Reference &ref);
  void follow_motion(const typename Tracker::MotionPoint &point);
  void begin_stop(Outcome outcome);
  void end_motion(Outcome outcome);
  void report(uint64_t id, Outcome outcome);

  Config config_{};
  Buffer buffer_{};
  Tracker tracker_{};
  State state_{State::kHolding};
  FinishReason finish_reason_{FinishReason::kNone};
  bool finish_requested_{false};
  // Newest stamp of an accepted sample, or the start of the run: base of the command timeout
  int64_t last_activity_ns_{0};
  int64_t last_playback_ns_{0};
  uint64_t gap_cycles_{0};

  Motion motion_{};
  double motion_time_{0.0};
  bool motion_active_{false};
  Request pending_{};
  bool has_pending_{false};
  Outcome stop_outcome_{Outcome::kNone};

  Counters counters_{};
  MotionStatus motion_status_{};
};

using JointPositionPipeline = Pipeline<JointTracker>;
using JointVelocityPipeline = Pipeline<JointVelocityTracker>;
using JointTorquePipeline = Pipeline<JointTorqueTracker>;
using CartesianPosePipeline = Pipeline<CartesianPoseTracker>;
using CartesianVelocityPipeline = Pipeline<CartesianVelocityTracker>;

// Instantiated in pipeline.cpp
extern template class Pipeline<JointTracker>;
extern template class Pipeline<JointVelocityTracker>;
extern template class Pipeline<JointTorqueTracker>;
extern template class Pipeline<CartesianPoseTracker>;
extern template class Pipeline<CartesianVelocityTracker>;

}  // namespace panda_bridge
