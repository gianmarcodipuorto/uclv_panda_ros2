#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

namespace panda_bridge {

// Settings applied to the robot when the bridge connects.
struct RobotSettings {
  std::string robot_ip;
  std::array<double, 7> lower_torque_thresholds{};
  std::array<double, 7> upper_torque_thresholds{};
  std::array<double, 6> lower_force_thresholds{};
  std::array<double, 6> upper_force_thresholds{};
  std::array<double, 7> joint_impedance{};
  std::array<double, 6> cartesian_impedance{};
  double load_mass{0.0};
  std::array<double, 3> F_x_Cload{};
  std::array<double, 9> load_inertia{};
  // Column-major nominal end effector to end effector transform (franka::Robot::setEE)
  std::array<double, 16> NE_T_EE{};
};

// Seam around franka::Robot: the bridges talk to the robot only through this interface, so that
// their lifecycle and threading can be tested with FakeRobotBackend, without a robot.
class RobotBackend {
 public:
  using JointPositionsCallback =
      std::function<franka::JointPositions(const franka::RobotState &, franka::Duration)>;
  using JointVelocitiesCallback =
      std::function<franka::JointVelocities(const franka::RobotState &, franka::Duration)>;
  using TorquesCallback = std::function<franka::Torques(const franka::RobotState &, franka::Duration)>;
  using CartesianPoseCallback =
      std::function<franka::CartesianPose(const franka::RobotState &, franka::Duration)>;
  using CartesianVelocitiesCallback =
      std::function<franka::CartesianVelocities(const franka::RobotState &, franka::Duration)>;

  virtual ~RobotBackend() = default;

  virtual franka::RobotState read_once() = 0;
  // Blocking control loops, return when the callback finishes the motion.
  virtual void control_joint_positions(const JointPositionsCallback &callback, franka::ControllerMode mode,
                                       bool limit_rate, double cutoff_frequency) = 0;
  virtual void control_joint_velocities(const JointVelocitiesCallback &callback, franka::ControllerMode mode,
                                        bool limit_rate, double cutoff_frequency) = 0;
  // Torques without gravity: the robot compensates it
  virtual void control_torques(const TorquesCallback &callback, bool limit_rate, double cutoff_frequency) = 0;
  virtual void control_cartesian_pose(const CartesianPoseCallback &callback, franka::ControllerMode mode,
                                      bool limit_rate, double cutoff_frequency) = 0;
  virtual void control_cartesian_velocities(const CartesianVelocitiesCallback &callback, franka::ControllerMode mode,
                                            bool limit_rate, double cutoff_frequency) = 0;
  virtual void automatic_error_recovery() = 0;
  // Aborts a running control loop from another thread.
  virtual void stop() = 0;
  // nullptr when no dynamic model is available
  virtual const franka::Model *model() const = 0;
};

// Connects to the robot through libfranka and applies the settings.
class FrankaRobotBackend : public RobotBackend {
 public:
  explicit FrankaRobotBackend(const RobotSettings &settings);

  franka::RobotState read_once() override;
  void control_joint_positions(const JointPositionsCallback &callback, franka::ControllerMode mode, bool limit_rate,
                               double cutoff_frequency) override;
  void control_joint_velocities(const JointVelocitiesCallback &callback, franka::ControllerMode mode,
                                bool limit_rate, double cutoff_frequency) override;
  void control_torques(const TorquesCallback &callback, bool limit_rate, double cutoff_frequency) override;
  void control_cartesian_pose(const CartesianPoseCallback &callback, franka::ControllerMode mode, bool limit_rate,
                              double cutoff_frequency) override;
  void control_cartesian_velocities(const CartesianVelocitiesCallback &callback, franka::ControllerMode mode,
                                    bool limit_rate, double cutoff_frequency) override;
  void automatic_error_recovery() override;
  void stop() override;
  const franka::Model *model() const override;

 private:
  franka::Robot robot_;
  std::optional<franka::Model> model_;
};

// Test-only robot: 1 kHz loop on the wall clock. Position and velocity commands are followed
// exactly, joint and Cartesian ones (the Cartesian commands move the end effector only, there is
// no kinematics); torques drive one decoupled inertia per joint, gravity compensated as on the
// robot. No dynamic model. Optionally it mimics the real robot: packets lost with the given probability
// (the next callback sees a 2 ms period), and a measured position offset from the commanded one
// (tracking error of the internal controller).
class FakeRobotBackend : public RobotBackend {
 public:
  explicit FakeRobotBackend(const std::array<double, 7> &initial_q, double packet_loss = 0.0,
                            double tracking_offset = 0.0);

  franka::RobotState read_once() override;
  void control_joint_positions(const JointPositionsCallback &callback, franka::ControllerMode mode, bool limit_rate,
                               double cutoff_frequency) override;
  void control_joint_velocities(const JointVelocitiesCallback &callback, franka::ControllerMode mode,
                                bool limit_rate, double cutoff_frequency) override;
  void control_torques(const TorquesCallback &callback, bool limit_rate, double cutoff_frequency) override;
  void control_cartesian_pose(const CartesianPoseCallback &callback, franka::ControllerMode mode, bool limit_rate,
                              double cutoff_frequency) override;
  void control_cartesian_velocities(const CartesianVelocitiesCallback &callback, franka::ControllerMode mode,
                                    bool limit_rate, double cutoff_frequency) override;
  void automatic_error_recovery() override {}
  void stop() override { stop_requested_.store(true); }
  const franka::Model *model() const override { return nullptr; }

 private:
  // Runs the loop; apply(command, period) moves the robot by one period under the command.
  template <typename Command, typename Apply>
  void run(const std::function<Command(const franka::RobotState &, franka::Duration)> &callback, Apply apply);
  void set_measured_from_commanded();
  void apply_elbow(const std::array<double, 2> &elbow, franka::Duration period);

  franka::RobotState state_{};
  std::atomic<bool> stop_requested_{false};
  double packet_loss_;
  double tracking_offset_;
};

}  // namespace panda_bridge
