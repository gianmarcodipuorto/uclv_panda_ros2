#pragma once

// Limits of the FER (Panda) arm. Kept free of ROS dependencies so that the real-time
// pipelines can use them; constants.hpp re-exports them.
#include <string>

#include <Eigen/Core>

namespace panda_constants {
const std::string panda_model_effort{"/models/panda/panda_fer.urdf"};

//i limiti sono quelli del panda
const Eigen::Vector<double, 7> effort_limits{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

const Eigen::Vector<double, 7> effort_speed_limits{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0};

const Eigen::Vector<double, 7> joint_min_limits = Eigen::Vector<double, 7>{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};

const Eigen::Vector<double, 7> joint_max_limits = Eigen::Vector<double, 7>{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

const Eigen::Vector<double, 7> velocity_limits = Eigen::Vector<double, 7>{2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};

const Eigen::Vector<double, 7> acceleration_limits = Eigen::Vector<double, 7>{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};

const Eigen::Vector<double, 2> cartesian_velocity_limits = Eigen::Vector<double, 2>{1.7,2.5}; //m/s and rad/s

} // namespace panda_constants
