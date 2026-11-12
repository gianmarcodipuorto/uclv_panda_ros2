#include "panda_utils/bridge/gripper_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <franka/exception.h>

#include "panda_utils/constants.hpp"

namespace panda_bridge {

namespace {

using namespace std::chrono_literals;

// Grasp tolerance on the expected object width (libfranka default) [m]
constexpr double kEpsilon = 0.005;
// Outer tolerance of a CLOSE without an expected width: any final opening counts as grasped [m]
constexpr double kAnyWidth = 1.0;
// A command that should move the fingers by more than this, and moves them by less, failed [m]
constexpr double kMinTravel = 0.001;
// Fingers closer than this to the maximum width rest on the outer end stop [m]
constexpr double kEndStopMargin = 0.001;
// Position move that takes the fingers off the outer end stop before a grasp [m]
constexpr double kUnstick = 0.002;
constexpr auto kStatePublishPeriod = 20ms;
constexpr auto kFeedbackPeriod = 100ms;

std::string mm(double width) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f mm", width * 1e3);
  return buffer;
}

const char *command_name(uint8_t command) {
  switch (command) {
    case GripperServer::Gripper::Goal::OPEN:
      return "open";
    case GripperServer::Gripper::Goal::CLOSE:
      return "close";
    case GripperServer::Gripper::Goal::MOVE:
      return "move";
    case GripperServer::Gripper::Goal::HOMING:
      return "homing";
    default:
      return "unknown";
  }
}

}  // namespace

GripperServer::GripperServer(rclcpp_lifecycle::LifecycleNode &node, std::unique_ptr<GripperBackend> backend,
                             const Params &params, const rclcpp::CallbackGroup::SharedPtr &group)
    : logger_(node.get_logger().get_child("gripper")),
      clock_(node.get_clock()),
      backend_(std::move(backend)),
      params_(params),
      group_(group) {
  reading_ = backend_->read_once();
  reading_count_ = 1;

  // Plain publisher, as the robot state: published while the bridge is inactive too
  auto parameters = node.get_node_parameters_interface();
  auto topics = node.get_node_topics_interface();
  state_pub_ = rclcpp::create_publisher<sensor_msgs::msg::JointState>(
      parameters, topics, std::string("/panda/gripper_state"), panda_interface_names::DEFAULT_TOPIC_QOS());
  action_server_ = rclcpp_action::create_server<Gripper>(
      node.get_node_base_interface(), node.get_node_clock_interface(), node.get_node_logging_interface(),
      node.get_node_waitables_interface(), "/panda/gripper",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Gripper::Goal> goal) { return on_goal(*goal); },
      [this](const std::shared_ptr<GoalHandle> handle) { return on_cancel(handle); },
      [this](const std::shared_ptr<GoalHandle> handle) { on_accepted(handle); }, rcl_action_server_get_default_options(),
      group);

  state_thread_ = std::thread(&GripperServer::state_thread_main, this);
  worker_thread_ = std::thread(&GripperServer::worker_thread_main, this);
}

GripperServer::~GripperServer() {
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    worker_exit_ = true;
    stop_running_command();
  }
  goal_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  state_exit_ = true;
  if (state_thread_.joinable()) {
    state_thread_.join();
  }
  action_server_.reset();
  state_pub_.reset();
}

rclcpp_action::GoalResponse GripperServer::on_goal(const Gripper::Goal &goal) {
  const GripperReading reading = latest_reading();
  const auto reject = [this](const std::string &why) {
    RCLCPP_WARN(logger_, "Goal rejected: %s", why.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  };
  if (goal.command > Gripper::Goal::HOMING) {
    return reject("unknown command " + std::to_string(goal.command) + " (0 open, 1 close, 2 move, 3 homing)");
  }
  if (!std::isfinite(goal.speed) || goal.speed < 0.0) {
    return reject("speed must be positive, or 0 for the default");
  }
  if (!std::isfinite(goal.force) || goal.force < 0.0) {
    return reject("force must be positive, or 0 for the default");
  }
  const bool needs_width = goal.command == Gripper::Goal::OPEN || goal.command == Gripper::Goal::MOVE ||
                           goal.command == Gripper::Goal::CLOSE;
  if (needs_width && !(reading.max_width > 0.0)) {
    return reject("maximum width unknown: run a homing first (command 3)");
  }
  if ((goal.command == Gripper::Goal::MOVE || goal.command == Gripper::Goal::CLOSE) &&
      (!std::isfinite(goal.width) || goal.width < 0.0 || goal.width > reading.max_width)) {
    return reject("width must be in [0, " + mm(reading.max_width) + "]");
  }
  if (goal.command == Gripper::Goal::CLOSE && (goal.force > 0.0 ? goal.force : params_.force) > kMaxForce) {
    return reject("force above the continuous force of the Franka Hand (70 N)");
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperServer::on_cancel(const std::shared_ptr<GoalHandle> &handle) {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (handle == running_) {
    stop_running_command();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperServer::on_accepted(const std::shared_ptr<GoalHandle> &handle) {
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (pending_) {
      auto result = std::make_shared<Gripper::Result>();
      result->width = latest_reading().width;
      result->message = "preempted by a newer goal";
      pending_->abort(result);
    }
    pending_ = handle;
    // The newest goal wins: stop the command running now
    if (running_) {
      stop_running_command();
    }
  }
  goal_cv_.notify_all();
}

void GripperServer::stop_running_command() {
  if (!running_) {
    return;
  }
  try {
    backend_->stop();
  } catch (const franka::Exception &e) {
    RCLCPP_WARN(logger_, "Stop failed: %s", e.what());
  }
}

void GripperServer::worker_thread_main() {
  std::unique_lock<std::mutex> lock(goal_mutex_);
  while (true) {
    goal_cv_.wait(lock, [this] { return worker_exit_ || pending_ != nullptr; });
    if (worker_exit_) {
      if (pending_) {
        auto result = std::make_shared<Gripper::Result>();
        result->message = "the bridge is shutting down";
        pending_->abort(result);
        pending_.reset();
      }
      return;
    }
    running_ = std::move(pending_);
    pending_.reset();
    const std::shared_ptr<GoalHandle> handle = running_;
    lock.unlock();

    auto result = std::make_shared<Gripper::Result>();
    const Outcome outcome = execute(handle, *result);

    lock.lock();
    // Only a command that went wrong is a warning: not one canceled or replaced on request
    bool failed = false;
    // A command that completed succeeds even if a cancel arrived meanwhile
    if (outcome == Outcome::kSucceeded) {
      handle->succeed(result);
    } else if (outcome == Outcome::kCanceled || handle->is_canceling()) {
      result->message = "canceled";
      handle->canceled(result);
    } else {
      if (pending_) {
        result->message = "preempted by a newer goal";
      } else if (worker_exit_) {
        result->message = "stopped: the bridge is shutting down";
      } else {
        failed = true;
      }
      handle->abort(result);
    }
    if (failed) {
      RCLCPP_WARN(logger_, "%s failed: %s", command_name(handle->get_goal()->command), result->message.c_str());
    } else {
      RCLCPP_INFO(logger_, "%s: %s", command_name(handle->get_goal()->command), result->message.c_str());
    }
    running_.reset();
  }
}

GripperServer::Outcome GripperServer::execute(const std::shared_ptr<GoalHandle> &handle, Gripper::Result &result) {
  const auto goal = handle->get_goal();
  if (handle->is_canceling()) {
    result.width = latest_reading().width;
    return Outcome::kCanceled;
  }
  const double speed = goal->speed > 0.0 ? goal->speed : params_.speed;
  const double force = goal->force > 0.0 ? goal->force : params_.force;
  const bool any_width = !(goal->width > 0.0);
  GripperReading before = latest_reading();

  bool ok = false;
  std::string error;
  try {
    switch (goal->command) {
      case Gripper::Goal::OPEN:
        ok = backend_->move(before.max_width, speed);
        break;
      case Gripper::Goal::CLOSE:
        // On the outer end stop the fingers may stick: a grasp, force limited, then does not start
        // them, and the hand reports a grasp where they are. A short position move frees them.
        if (before.width > before.max_width - kEndStopMargin) {
          backend_->move(std::max(0.0, before.width - kUnstick), speed);
          before = next_reading();
        }
        ok = backend_->grasp(any_width ? 0.0 : goal->width, speed, force, kEpsilon, any_width ? kAnyWidth : kEpsilon);
        break;
      case Gripper::Goal::MOVE:
        ok = backend_->move(goal->width, speed);
        break;
      case Gripper::Goal::HOMING:
        ok = backend_->homing();
        break;
      default:
        error = "unknown command";
    }
  } catch (const franka::Exception &e) {
    error = e.what();
  }

  GripperReading after = next_reading();
  result.width = after.width;
  if (!error.empty()) {
    result.success = false;
    result.message = error;
    return Outcome::kAborted;
  }

  // Fingers that should have moved and did not: stuck, whatever the hand reports
  double target = before.width;
  if (goal->command == Gripper::Goal::OPEN) {
    target = before.max_width;
  } else if (goal->command == Gripper::Goal::CLOSE) {
    target = any_width ? 0.0 : goal->width;
  } else if (goal->command == Gripper::Goal::MOVE) {
    target = goal->width;
  }
  if (std::abs(target - before.width) > kMinTravel && std::abs(after.width - before.width) < kMinTravel) {
    result.success = false;
    result.message = "the fingers did not move: try again, or run a homing (command 3)";
    return Outcome::kAborted;
  }

  // Nothing between the fingers: stop squeezing them against each other, they stay closed
  if (goal->command == Gripper::Goal::CLOSE && after.width < kEmptyWidth) {
    try {
      backend_->stop();
    } catch (const franka::Exception &e) {
      RCLCPP_WARN(logger_, "Could not release the force of the closed fingers: %s", e.what());
    }
  }

  result.success = ok;
  switch (goal->command) {
    case Gripper::Goal::OPEN:
      result.message = ok ? "open at " + mm(after.width) : "open stopped at " + mm(after.width);
      break;
    case Gripper::Goal::CLOSE:
      if (any_width) {
        result.message = after.width < kEmptyWidth ? "closed: nothing between the fingers"
                                                   : "holding an object " + mm(after.width) + " wide";
      } else {
        result.message = ok ? "holding an object " + mm(after.width) + " wide"
                            : "no object of " + mm(goal->width) + ": fingers at " + mm(after.width);
      }
      break;
    case Gripper::Goal::MOVE:
      result.message = ok ? "at " + mm(after.width) : "stopped at " + mm(after.width);
      break;
    case Gripper::Goal::HOMING:
      result.message = ok ? "homing done, maximum width " + mm(after.max_width) : "homing failed";
      break;
    default:
      break;
  }
  return ok ? Outcome::kSucceeded : Outcome::kAborted;
}

void GripperServer::state_thread_main() {
  auto last_publish = std::chrono::steady_clock::time_point{};
  auto last_feedback = std::chrono::steady_clock::time_point{};
  while (!state_exit_) {
    GripperReading reading;
    try {
      reading = backend_->read_once();
    } catch (const franka::Exception &e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Cannot read the gripper state: %s", e.what());
      std::this_thread::sleep_for(100ms);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      reading_ = reading;
      ++reading_count_;
    }
    state_cv_.notify_all();

    const auto now = std::chrono::steady_clock::now();
    if (now - last_publish >= kStatePublishPeriod) {
      last_publish = now;
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = clock_->now();
      msg.name = params_.joint_names;
      msg.position.assign(2, reading.width / 2.0);
      state_pub_->publish(msg);
    }
    if (now - last_feedback >= kFeedbackPeriod) {
      last_feedback = now;
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (running_ && running_->is_executing()) {
        auto feedback = std::make_shared<Gripper::Feedback>();
        feedback->width = reading.width;
        running_->publish_feedback(feedback);
      }
    }
  }
}

GripperReading GripperServer::latest_reading() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return reading_;
}

GripperReading GripperServer::next_reading() {
  std::unique_lock<std::mutex> lock(state_mutex_);
  const uint64_t seen = reading_count_;
  state_cv_.wait_for(lock, 500ms, [this, seen] { return reading_count_ > seen + 1; });
  return reading_;
}

}  // namespace panda_bridge
