#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <franka/gripper.h>

namespace panda_bridge {

struct GripperReading {
  double width{0.0};
  double max_width{0.0};
  bool is_grasped{false};
  uint16_t temperature{0};
};

// Seam around franka::Gripper, as RobotBackend is for the robot. The commands block until the
// fingers stop; stop() may be called from another thread while one runs, and makes it throw
// franka::CommandException. read_once() must not be called from two threads at the same time.
class GripperBackend {
 public:
  virtual ~GripperBackend() = default;

  // Blocks until the next state update of the gripper
  virtual GripperReading read_once() = 0;
  virtual bool homing() = 0;
  virtual bool move(double width, double speed) = 0;
  virtual bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) = 0;
  virtual bool stop() = 0;
};

// The Franka Hand through libfranka: a connection of its own, independent of the robot's
class FrankaGripperBackend : public GripperBackend {
 public:
  explicit FrankaGripperBackend(const std::string &robot_ip) : gripper_(robot_ip) {}

  GripperReading read_once() override;
  bool homing() override { return gripper_.homing(); }
  bool move(double width, double speed) override { return gripper_.move(width, speed); }
  bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) override {
    return gripper_.grasp(width, speed, force, epsilon_inner, epsilon_outer);
  }
  bool stop() override { return gripper_.stop(); }

 private:
  franka::Gripper gripper_;
};

// Test-only Franka Hand: the fingers move at the commanded speed on the wall clock and an object
// of object_width (0 = none) stops them while closing. A stop aborts a running command as on the
// real gripper; with no command running it releases the grasp.
class FakeGripperBackend : public GripperBackend {
 public:
  explicit FakeGripperBackend(double object_width = 0.0, double max_width = 0.08)
      : width_(max_width), max_width_(max_width), object_width_(object_width) {}

  GripperReading read_once() override;
  bool homing() override;
  bool move(double width, double speed) override;
  bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) override;
  bool stop() override;

 private:
  // Moves the fingers towards target, stopping on the object while closing. Throws if stopped.
  void travel(double target, double speed);

  std::mutex mutex_;
  double width_;
  double max_width_;
  bool is_grasped_{false};
  double object_width_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace panda_bridge
