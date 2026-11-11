#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "uclv_panda_interfaces/msg/bridge_status.hpp"

#include "panda_utils/bridge/pipeline_types.hpp"
#include "panda_utils/bridge/rt_exchange.hpp"
#include "panda_utils/bridge/types.hpp"

namespace panda_bridge {

// Pieces of the ROS side shared by the joint and the Cartesian bridges.

// State of the real-time loop, published every cycle for the ROS side.
template <typename Position>
struct PipelineSnapshot {
  bool valid{false};
  PipelineTypes::State state{PipelineTypes::State::kHolding};
  PipelineTypes::Counters counters{};
  PipelineTypes::MotionStatus motion{};
  // Where the internal motions start from
  Position position{};
  uint64_t tracker_errors{0};
};

// Events of the real-time loop, logged by the publisher thread.
enum BridgeEvent : uint16_t {
  kStreamStarted,
  kStreamStalled,
  kStreamRejected,
  kMotionStarted,
  kMotionFinished,
  kNonFiniteCommand,
  kTrackerError,
};

// Real-time: pushes the events between two snapshots of the loop.
template <typename Position, typename Queue>
void push_pipeline_events(Queue &events, const PipelineSnapshot<Position> &previous,
                          const PipelineSnapshot<Position> &current, double stall_timeout_ms) {
  auto push = [&events](BridgeEvent code, uint64_t id, double value) { (void)events.push(RtEvent{code, id, value}); };
  if (current.counters.stream_starts != previous.counters.stream_starts) {
    push(kStreamStarted, current.counters.stream_starts, 0.0);
  }
  if (current.counters.stalls != previous.counters.stalls) {
    push(kStreamStalled, current.counters.stalls, stall_timeout_ms);
  }
  if (current.counters.rejected_jumps != previous.counters.rejected_jumps) {
    push(kStreamRejected, current.counters.rejected_jumps, current.counters.last_rejected_jump);
  }
  if (current.motion.active_id != 0 && current.motion.active_id != previous.motion.active_id) {
    push(kMotionStarted, current.motion.active_id, current.motion.duration);
  }
  if (current.motion.last_finished_id != previous.motion.last_finished_id) {
    push(kMotionFinished, current.motion.last_finished_id, static_cast<double>(current.motion.last_outcome));
  }
  if (current.tracker_errors != previous.tracker_errors) {
    push(kTrackerError, current.tracker_errors, 0.0);
  }
}

inline const char *outcome_name(PipelineTypes::Outcome outcome) {
  switch (outcome) {
    case PipelineTypes::Outcome::kSucceeded:
      return "succeeded";
    case PipelineTypes::Outcome::kCanceled:
      return "canceled";
    case PipelineTypes::Outcome::kAborted:
      return "aborted";
    case PipelineTypes::Outcome::kNone:
    default:
      return "finished";
  }
}

// Logs an event of the loop. quantity names the commands ("joint positions"), jump_unit the
// distance of a rejected stream ("rad", "m"); rejections are logged at most every 2 s.
inline void log_bridge_event(const rclcpp::Logger &logger, const RtEvent &event, const char *quantity,
                             const char *jump_unit, int64_t &last_rejection_log_ns) {
  switch (event.code) {
    case kStreamStarted:
      RCLCPP_INFO(logger, "Following the command stream");
      break;
    case kStreamStalled:
      RCLCPP_WARN(logger, "Command stream interrupted for more than %.0f ms: smooth stop", event.value);
      break;
    case kStreamRejected: {
      const int64_t now = wall_clock_ns();
      if (now - last_rejection_log_ns > 2'000'000'000) {
        last_rejection_log_ns = now;
        RCLCPP_WARN(logger, "Command stream rejected: it starts %.3f %s away from the current command (max_jump)",
                    event.value, jump_unit);
      }
      break;
    }
    case kMotionStarted:
      if (std::isinf(event.value)) {
        RCLCPP_INFO(logger, "Motion %lu started, until stopped", static_cast<unsigned long>(event.id));
      } else {
        RCLCPP_INFO(logger, "Motion %lu started, duration %.2f s", static_cast<unsigned long>(event.id), event.value);
      }
      break;
    case kMotionFinished:
      RCLCPP_INFO(logger, "Motion %lu %s", static_cast<unsigned long>(event.id),
                  outcome_name(static_cast<PipelineTypes::Outcome>(static_cast<int>(event.value))));
      break;
    case kNonFiniteCommand:
      RCLCPP_ERROR(logger, "Non-finite %s command: motion finished", quantity);
      break;
    case kTrackerError:
      RCLCPP_ERROR(logger, "Trajectory generation failed (%lu times): holding the command",
                   static_cast<unsigned long>(event.id));
      break;
    default:
      break;
  }
}

// Time base of a command sample: its header stamp, or the time of arrival for unstamped samples
// and for stamps of another clock (more than 50 ms in the future or 1 s in the past).
inline int64_t playback_stamp(int64_t stamp_ns, int64_t received_ns) {
  constexpr int64_t kMaxFutureStampNs = 50'000'000;
  constexpr int64_t kMaxStampAgeNs = 1'000'000'000;
  if (stamp_ns == 0 || stamp_ns > received_ns + kMaxFutureStampNs || stamp_ns < received_ns - kMaxStampAgeNs) {
    return received_ns;
  }
  return stamp_ns;
}

// State, motion and counters of the status, from the snapshot of the loop.
template <typename Position>
void fill_pipeline_status(uclv_panda_interfaces::msg::BridgeStatus &status, const PipelineSnapshot<Position> &snap,
                          bool running, bool internal_mode) {
  using BridgeStatus = uclv_panda_interfaces::msg::BridgeStatus;
  status.mode = internal_mode ? "internal" : "external";
  if (running && snap.valid) {
    switch (snap.state) {
      case PipelineTypes::State::kHolding:
        status.state = BridgeStatus::STATE_HOLDING;
        break;
      case PipelineTypes::State::kMoving:
        status.state = BridgeStatus::STATE_MOVING;
        break;
      case PipelineTypes::State::kStopping:
      case PipelineTypes::State::kFinished:
        status.state = BridgeStatus::STATE_STOPPING;
        break;
    }
    if (snap.motion.active_id != 0) {
      status.motion = snap.motion.active_name.c_str();
    } else if (snap.state == PipelineTypes::State::kMoving && !internal_mode) {
      status.motion = "stream";
    }
  }
  status.stalls = snap.counters.stalls;
  status.limited_cycles = snap.counters.limited_cycles;
  status.extrapolated_cycles = snap.counters.extrapolated_cycles;
  status.playout_margin_min_ms = static_cast<double>(snap.counters.playout_margin_min_ns) * 1e-6;
  status.rejected_commands = snap.counters.rejected_jumps;
}

// Rate and largest interval of the incoming command samples, for the status.
class StreamStatistics {
 public:
  void on_sample(int64_t received_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_receive_ns_ != 0) {
      window_max_interval_ns_ = std::max(window_max_interval_ns_, received_ns - last_receive_ns_);
    }
    last_receive_ns_ = received_ns;
    if (window_start_ns_ == 0) {
      window_start_ns_ = received_ns;
    }
    ++window_samples_;
    if (received_ns - window_start_ns_ >= 1'000'000'000) {
      rate_hz_ = static_cast<double>(window_samples_) / ns_to_s(received_ns - window_start_ns_);
      interval_max_ms_ = static_cast<double>(window_max_interval_ns_) * 1e-6;
      window_start_ns_ = received_ns;
      window_samples_ = 0;
      window_max_interval_ns_ = 0;
    }
  }

  // Zero once the stream has been silent for a second
  void fill(uclv_panda_interfaces::msg::BridgeStatus &status) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool alive = last_receive_ns_ != 0 && wall_clock_ns() - last_receive_ns_ < 1'000'000'000;
    status.command_rate_hz = alive ? rate_hz_ : 0.0;
    status.command_interval_max_ms = alive ? interval_max_ms_ : 0.0;
  }

 private:
  std::mutex mutex_;
  int64_t last_receive_ns_{0};
  int64_t window_start_ns_{0};
  uint64_t window_samples_{0};
  int64_t window_max_interval_ns_{0};
  double rate_hz_{0.0};
  double interval_max_ms_{0.0};
};

namespace detail {
// result.message, for the actions whose result has one
template <typename Result>
auto set_message(Result &result, const char *message, int) -> decltype(result.message = message, void()) {
  result.message = message;
}
template <typename Result>
void set_message(Result &, const char *, long) {}
}  // namespace detail

// Action goals executed as internal motions: each goal is bound to the id of its motion, and
// completed from the state of the motions published by the loop.
template <typename Action>
class GoalTracker {
 public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  void add(const std::shared_ptr<GoalHandle> &handle, uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    goals_.push_back({handle, id});
  }

  bool empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.empty();
  }

  // Motions of a goal, to cancel them
  std::vector<uint64_t> ids_of(const std::shared_ptr<GoalHandle> &handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint64_t> ids;
    for (const auto &goal : goals_) {
      if (goal.handle == handle) {
        ids.push_back(goal.id);
      }
    }
    return ids;
  }

  // Succeeds, cancels or aborts the goals whose motion is over; feedback for the active one.
  void update(const PipelineTypes::MotionStatus &motion, bool snapshot_valid, bool running,
              const rclcpp::Logger &logger) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = goals_.begin(); it != goals_.end();) {
      const uint64_t id = it->id;
      const auto &handle = it->handle;
      auto result = std::make_shared<typename Action::Result>();
      result->completed = false;
      bool done = true;
      try {
        if (snapshot_valid && motion.last_finished_id == id) {
          result->completed = motion.last_outcome == PipelineTypes::Outcome::kSucceeded;
          finish(handle, result, outcome_name(motion.last_outcome));
        } else if (!running || (snapshot_valid && (motion.last_finished_id > id || motion.active_id > id))) {
          finish(handle, result, running ? "replaced by another motion" : "control loop stopped");
        } else {
          done = false;
          if (snapshot_valid && motion.active_id == id) {
            auto feedback = std::make_shared<typename Action::Feedback>();
            feedback->time_left = std::max(0.0, motion.duration - motion.elapsed);
            handle->publish_feedback(feedback);
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(logger, "Goal %lu: %s", static_cast<unsigned long>(id), e.what());
        done = true;
      }
      it = done ? goals_.erase(it) : it + 1;
    }
  }

  void abort_all() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &goal : goals_) {
      try {
        auto result = std::make_shared<typename Action::Result>();
        result->completed = false;
        detail::set_message(*result, "bridge cleaned up", 0);
        goal.handle->abort(result);
      } catch (const std::exception &) {
        // already terminated
      }
    }
    goals_.clear();
  }

 private:
  struct Tracked {
    std::shared_ptr<GoalHandle> handle;
    uint64_t id;
  };

  static void finish(const std::shared_ptr<GoalHandle> &handle, const std::shared_ptr<typename Action::Result> &result,
                     const char *message) {
    detail::set_message(*result, message, 0);
    if (result->completed) {
      handle->succeed(result);
    } else if (handle->is_canceling()) {
      handle->canceled(result);
    } else {
      handle->abort(result);
    }
  }

  std::mutex mutex_;
  std::vector<Tracked> goals_;
};

}  // namespace panda_bridge
