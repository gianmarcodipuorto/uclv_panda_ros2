#include "panda_utils/bridge/gripper_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper_state.h>

namespace panda_bridge {

GripperReading FrankaGripperBackend::read_once() {
  const franka::GripperState state = gripper_.readOnce();
  GripperReading reading;
  reading.width = state.width;
  reading.max_width = state.max_width;
  reading.is_grasped = state.is_grasped;
  reading.temperature = state.temperature;
  return reading;
}

namespace {

constexpr auto kFakeStatePeriod = std::chrono::milliseconds(20);
constexpr auto kFakeStep = std::chrono::milliseconds(1);

}  // namespace

GripperReading FakeGripperBackend::read_once() {
  std::this_thread::sleep_for(kFakeStatePeriod);
  std::lock_guard<std::mutex> lock(mutex_);
  GripperReading reading;
  reading.width = width_;
  reading.max_width = max_width_;
  reading.is_grasped = is_grasped_;
  reading.temperature = 30;
  return reading;
}

void FakeGripperBackend::travel(double target, double speed, bool force_limited) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool on_outer_end_stop = width_ >= max_width_ - 1e-9;
    if (jam_ == Jam::kAlways || (jam_ == Jam::kAtOuterEndStop && force_limited && on_outer_end_stop)) {
      // The motor stalls at once: the hand takes it for a contact where the fingers are
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      return;
    }
  }
  running_ = true;
  stop_requested_ = false;
  const double step = speed * std::chrono::duration<double>(kFakeStep).count();
  while (true) {
    if (stop_requested_.exchange(false)) {
      running_ = false;
      throw franka::CommandException("libfranka gripper: Command aborted!");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Closing on the object: the fingers stop on it
      const double floor = (object_width_ > 0.0 && width_ >= object_width_) ? object_width_ : 0.0;
      const double goal = std::max(target, floor);
      if (std::abs(width_ - goal) <= step) {
        width_ = goal;
        break;
      }
      width_ += (goal > width_) ? step : -step;
    }
    std::this_thread::sleep_for(kFakeStep);
  }
  running_ = false;
}

bool FakeGripperBackend::homing() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_grasped_ = false;
  }
  travel(max_width_, 0.1, false);
  return true;
}

bool FakeGripperBackend::move(double width, double speed) {
  if (!(width >= 0.0 && width <= max_width_) || !(speed > 0.0)) {
    throw franka::CommandException("libfranka gripper: Command failed!");
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_grasped_ = false;
  }
  travel(width, speed, false);
  std::lock_guard<std::mutex> lock(mutex_);
  return std::abs(width_ - width) < 1e-9;
}

bool FakeGripperBackend::grasp(double width, double speed, double force, double epsilon_inner,
                               double epsilon_outer) {
  if (!(width >= 0.0 && width <= max_width_) || !(speed > 0.0) || !(force > 0.0)) {
    throw franka::CommandException("libfranka gripper: Command failed!");
  }
  travel(0.0, speed, true);
  std::lock_guard<std::mutex> lock(mutex_);
  is_grasped_ = width - epsilon_inner < width_ && width_ < width + epsilon_outer;
  return is_grasped_;
}

bool FakeGripperBackend::grasping() {
  std::lock_guard<std::mutex> lock(mutex_);
  return is_grasped_;
}

bool FakeGripperBackend::stop() {
  if (running_) {
    stop_requested_ = true;
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    is_grasped_ = false;
  }
  return true;
}

}  // namespace panda_bridge
