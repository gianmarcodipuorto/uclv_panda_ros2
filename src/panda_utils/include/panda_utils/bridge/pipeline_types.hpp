#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "panda_utils/bridge/franka_limits.hpp"

namespace panda_bridge {

struct MotionName {
  std::array<char, 32> text{};

  void set(const char *name) {
    text.fill('\0');
    std::strncpy(text.data(), name, text.size() - 1);
  }
  const char *c_str() const { return text.data(); }
};

// Request from the ROS side to the real-time loop.
template <typename Motion>
struct MotionRequest {
  enum class Kind : uint8_t { kStart, kCancel };

  Kind kind{Kind::kStart};
  // Motion identifier; for kCancel 0 cancels whatever motion is active or pending
  uint64_t id{0};
  Motion motion{};
  MotionName name{};
};

// Settings of the Cartesian pipelines.
struct CartesianSettings {
  CartesianLimits limits{fer_cartesian_limits()};
  // Lowest height (m) of the commanded end effector: critical_height plus a margin
  double min_height{0.11};
  // Largest distance between the command and a pose stream that (re)starts
  double max_jump{0.05};       // m
  double max_jump_angle{0.2};  // rad
  // false: the elbow is not commanded, the robot chooses it
  bool use_elbow{false};
  double elbow_margin{0.01};  // rad from the joint 3 limits
};

// Types shared by the pipelines.
struct PipelineTypes {
  enum class Mode : uint8_t { kExternal, kInternal };
  enum class State : uint8_t { kHolding, kMoving, kStopping, kFinished };
  enum class Outcome : uint8_t { kNone, kSucceeded, kCanceled, kAborted };
  enum class FinishReason : uint8_t { kNone, kRequested, kCommandTimeout, kSafety, kLimits };

  struct Config {
    Mode mode{Mode::kExternal};
    int64_t playout_delay_ns{5'000'000};
    // Half width of the window of samples fitted around the playback time
    int64_t fit_half_window_ns{3'000'000};
    // Gap after which the stream counts as interrupted; shorter gaps are extrapolated
    int64_t stall_timeout_ns{20'000'000};
    // Without samples for this long the motion is finished; 0 holds forever
    int64_t command_timeout_ns{0};
    // false: references beyond the limits stop the robot instead of being clamped
    bool clamp{true};
    // Joint pipelines. max_jump: largest distance (rad) between the command and a position stream
    // that (re)starts. Kinematic limits of the commands (positions, velocities) and of the
    // internal motions; the torque settings of the torque pipeline.
    double max_jump{0.2};
    double position_margin{0.01};
    JointLimits limits{fer_joint_limits()};
    TorqueSettings torque{fer_torque_settings()};
    // Cartesian pipelines
    CartesianSettings cartesian{};
  };

  struct Counters {
    uint64_t stalls{0};
    uint64_t stream_starts{0};
    uint64_t limited_cycles{0};
    // Cycles of the stream bridged by extrapolation, in gaps after which the stream went on
    uint64_t extrapolated_cycles{0};
    uint64_t rejected_jumps{0};
    double last_rejected_jump{0.0};
    // Smallest lead of a sample, at arrival, over the playback time that first needs it, since
    // the stream (re)started; negative when samples arrived late. Tells whether
    // playout_delay_ms is large enough.
    int64_t playout_margin_min_ns{0};
  };

  struct MotionStatus {
    uint64_t active_id{0};
    MotionName active_name{};
    double elapsed{0.0};
    double duration{0.0};
    uint64_t last_finished_id{0};
    Outcome last_outcome{Outcome::kNone};
  };
};

}  // namespace panda_bridge
