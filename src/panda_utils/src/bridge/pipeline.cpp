#include "panda_utils/bridge/pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace panda_bridge {

template <typename Tracker>
void Pipeline<Tracker>::configure(const Config &config) {
  config_ = config;
  tracker_.configure(config);
}

template <typename Tracker>
void Pipeline<Tracker>::reset(const Feedback &robot, int64_t now_ns) {
  buffer_.clear();
  tracker_.reset(robot);
  finish_reason_ = FinishReason::kNone;
  finish_requested_ = false;
  last_activity_ns_ = now_ns;
  last_playback_ns_ = 0;
  gap_cycles_ = 0;
  motion_time_ = 0.0;
  motion_active_ = false;
  has_pending_ = false;
  stop_outcome_ = Outcome::kNone;
  counters_ = Counters{};
  motion_status_ = MotionStatus{};
  // A control run starts with the robot at rest; brake first if it is not
  state_ = tracker_.at_rest() ? State::kHolding : State::kStopping;
}

template <typename Tracker>
void Pipeline<Tracker>::push_sample(const Sample &sample) {
  if (config_.mode != Mode::kExternal || !all_finite(sample.values)) {
    return;
  }
  // Orientations as continuous quaternions, so that the fit across samples is meaningful
  Sample aligned = sample;
  tracker_.align_sample(aligned.values, buffer_.empty() ? nullptr : &buffer_.newest().values);
  if (buffer_.push(aligned)) {
    last_activity_ns_ = std::max(last_activity_ns_, aligned.stamp_ns);
    if (last_playback_ns_ != 0) {
      // The next cycle plays back one cycle after the last one
      counters_.playout_margin_min_ns =
          std::min(counters_.playout_margin_min_ns, aligned.stamp_ns - (last_playback_ns_ + kCycleNs));
    }
  }
}

template <typename Tracker>
void Pipeline<Tracker>::request(const Request &request) {
  if (request.kind == Request::Kind::kCancel) {
    if (has_pending_ && (request.id == 0 || request.id == pending_.id)) {
      has_pending_ = false;
      report(pending_.id, Outcome::kCanceled);
    }
    if (motion_active_ && state_ == State::kMoving &&
        (request.id == 0 || request.id == motion_status_.active_id)) {
      begin_stop(Outcome::kCanceled);
    }
    return;
  }
  if (config_.mode != Mode::kInternal || finish_requested_ || state_ == State::kFinished) {
    report(request.id, Outcome::kAborted);
    return;
  }
  if (has_pending_) {
    report(pending_.id, Outcome::kAborted);
  }
  pending_ = request;
  has_pending_ = true;
  // A running motion is preempted: stop, then start the new one from rest
  if (state_ == State::kMoving && motion_active_) {
    begin_stop(Outcome::kAborted);
  }
}

template <typename Tracker>
void Pipeline<Tracker>::request_finish(FinishReason reason) {
  if (finish_requested_ || state_ == State::kFinished) {
    return;
  }
  finish_requested_ = true;
  finish_reason_ = reason;
  if (has_pending_) {
    has_pending_ = false;
    report(pending_.id, Outcome::kAborted);
  }
  if (state_ == State::kHolding) {
    state_ = State::kFinished;
  } else if (state_ == State::kMoving) {
    begin_stop(Outcome::kAborted);
  }
}

template <typename Tracker>
void Pipeline<Tracker>::step(int64_t now_ns) {
  last_playback_ns_ = now_ns - config_.playout_delay_ns;
  switch (state_) {
    case State::kHolding:
      step_holding(now_ns);
      break;
    case State::kMoving:
      step_moving(now_ns);
      break;
    case State::kStopping:
      step_stopping();
      break;
    case State::kFinished:
      tracker_.hold();
      break;
  }
}

template <typename Tracker>
void Pipeline<Tracker>::step_holding(int64_t now_ns) {
  if (config_.mode == Mode::kInternal) {
    tracker_.hold();
    if (has_pending_) {
      has_pending_ = false;
      start_motion(pending_);
    }
    return;
  }
  // One command per cycle: a stream that starts replaces the hold
  const auto ref = sample(now_ns);
  if (ref.status == Buffer::Status::kInterpolated && start_stream(ref, now_ns)) {
    return;
  }
  tracker_.hold();
  if (config_.command_timeout_ns > 0 && now_ns - last_activity_ns_ > config_.command_timeout_ns) {
    request_finish(FinishReason::kCommandTimeout);
  }
}

template <typename Tracker>
typename Pipeline<Tracker>::Buffer::Reference Pipeline<Tracker>::sample(int64_t now_ns) {
  const int64_t playback_ns = now_ns - config_.playout_delay_ns;
  if constexpr (Tracker::kSmoothStream) {
    // Gaps are bridged at constant acceleration: for velocity streams without the jerk of the fit
    return buffer_.sample_fit(playback_ns, config_.fit_half_window_ns, !Tracker::kStreamOfVelocities);
  } else {
    return buffer_.sample_linear(playback_ns);
  }
}

template <typename Tracker>
bool Pipeline<Tracker>::start_stream(const typename Buffer::Reference &ref, int64_t now_ns) {
  double distance = 0.0;
  if (tracker_.rejects_stream_start(ref.value, distance)) {
    ++counters_.rejected_jumps;
    counters_.last_rejected_jump = distance;
    buffer_.clear();
    return false;
  }
  if (!config_.clamp && !tracker_.reference_within_limits(ref.value, ref.rate)) {
    request_finish(FinishReason::kLimits);
    return false;
  }
  state_ = State::kMoving;
  ++counters_.stream_starts;
  counters_.playout_margin_min_ns = buffer_.newest().stamp_ns - (now_ns - config_.playout_delay_ns);
  gap_cycles_ = 0;
  follow_stream(ref);
  return true;
}

template <typename Tracker>
void Pipeline<Tracker>::step_moving(int64_t now_ns) {
  if (config_.mode == Mode::kInternal) {
    motion_time_ += kCycleTime;
    if (motion_.is_free()) {
      if constexpr (Tracker::kSupportsFreeMotion) {
        if (tracker_.follow_free()) {
          ++counters_.limited_cycles;
        }
      }
      motion_status_.elapsed = motion_time_;
      return;
    }
    typename Tracker::MotionPoint point;
    motion_.evaluate(motion_time_, point);
    follow_motion(point);
    motion_status_.elapsed = std::min(motion_time_, motion_.duration);
    if (motion_time_ >= motion_.duration && tracker_.at_rest()) {
      end_motion(Outcome::kSucceeded);
      state_ = State::kHolding;
    }
    return;
  }

  const auto ref = sample(now_ns);
  if (ref.status == Buffer::Status::kInterpolated) {
    counters_.extrapolated_cycles += gap_cycles_;
    gap_cycles_ = 0;
    if (!config_.clamp && !tracker_.reference_within_limits(ref.value, ref.rate)) {
      request_finish(FinishReason::kLimits);
      step_stopping();
      return;
    }
    follow_stream(ref);
    return;
  }
  if (ref.status == Buffer::Status::kStarved && ref.age_ns <= config_.stall_timeout_ns) {
    // Short gap: the buffer extrapolates the last samples (torques: holds the newest)
    ++gap_cycles_;
    follow_stream(ref);
    return;
  }
  // The stream ended or stalled. Never interpolate across the gap: resume from fresh samples only
  if (!tracker_.nearly_at_rest()) {
    ++counters_.stalls;
  }
  gap_cycles_ = 0;
  buffer_.clear();
  begin_stop(Outcome::kNone);
  step_stopping();
}

template <typename Tracker>
void Pipeline<Tracker>::step_stopping() {
  if (!tracker_.brake()) {
    return;
  }
  if (finish_requested_) {
    end_motion(Outcome::kAborted);
    state_ = State::kFinished;
    return;
  }
  end_motion(stop_outcome_ == Outcome::kNone ? Outcome::kAborted : stop_outcome_);
  state_ = State::kHolding;
  if (has_pending_) {
    has_pending_ = false;
    start_motion(pending_);
  }
}

template <typename Tracker>
void Pipeline<Tracker>::start_motion(const Request &request) {
  // The tracker starts the motion from the actual command, wherever the request assumed it to
  // be, checks its limits and stretches it to respect the kinematic limits
  Motion motion = request.motion;
  if (!tracker_.prepare_motion(motion)) {
    report(request.id, Outcome::kAborted);
    return;
  }
  motion_ = motion;
  motion_time_ = 0.0;
  motion_active_ = true;
  motion_status_.active_id = request.id;
  motion_status_.active_name = request.name;
  motion_status_.elapsed = 0.0;
  motion_status_.duration = motion.duration;
  stop_outcome_ = Outcome::kNone;
  state_ = State::kMoving;
}

template <typename Tracker>
void Pipeline<Tracker>::follow_stream(const typename Buffer::Reference &ref) {
  if (tracker_.follow_stream(ref.value, ref.rate, ref.rate_dot)) {
    ++counters_.limited_cycles;
  }
}

template <typename Tracker>
void Pipeline<Tracker>::follow_motion(const typename Tracker::MotionPoint &point) {
  if (tracker_.follow_motion(point)) {
    ++counters_.limited_cycles;
  }
}

template <typename Tracker>
void Pipeline<Tracker>::begin_stop(Outcome outcome) {
  state_ = State::kStopping;
  stop_outcome_ = outcome;
}

template <typename Tracker>
void Pipeline<Tracker>::end_motion(Outcome outcome) {
  if (!motion_active_) {
    return;
  }
  report(motion_status_.active_id, outcome);
  motion_active_ = false;
  motion_status_.active_id = 0;
  motion_status_.active_name.set("");
  motion_status_.elapsed = 0.0;
  motion_status_.duration = 0.0;
}

template <typename Tracker>
void Pipeline<Tracker>::report(uint64_t id, Outcome outcome) {
  motion_status_.last_finished_id = id;
  motion_status_.last_outcome = outcome;
}

template class Pipeline<JointTracker>;
template class Pipeline<JointVelocityTracker>;
template class Pipeline<JointTorqueTracker>;
template class Pipeline<CartesianPoseTracker>;
template class Pipeline<CartesianVelocityTracker>;

}  // namespace panda_bridge
