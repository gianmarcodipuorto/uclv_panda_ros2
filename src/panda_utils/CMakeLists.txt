cmake_minimum_required(VERSION 3.8)
project(panda_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uclv_panda_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# libfranka 0.9.x is the last release compatible with the FER (Panda), installed from source
find_package(Franka 0.9 REQUIRED)
find_package(ruckig REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Python support for GUI
find_package(ament_cmake_python REQUIRED)

#Include Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include)

# Shared by the bridges: command pipeline, robot and gripper backends, lifecycle base node
add_library(panda_bridge_core STATIC
  src/bridge/pipeline.cpp
  src/bridge/robot_backend.cpp
  src/bridge/gripper_backend.cpp
  src/bridge/gripper_server.cpp
  src/bridge/state_conversions.cpp
  src/bridge/bridge_base.cpp
  src/bridge/joint_bridge.cpp
  src/bridge/cartesian_bridge.cpp
)
set_target_properties(panda_bridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(panda_bridge_core PUBLIC include)
ament_target_dependencies(panda_bridge_core PUBLIC
  rclcpp rclcpp_lifecycle rclcpp_action lifecycle_msgs realtime_tools uclv_panda_interfaces
  sensor_msgs geometry_msgs std_srvs tf2_eigen)
target_link_libraries(panda_bridge_core PUBLIC Franka::Franka ruckig::ruckig)

add_executable(panda_gui  src/panda_gui.cpp)
ament_target_dependencies(panda_gui rclcpp rclcpp_lifecycle std_srvs geometry_msgs sensor_msgs)
target_link_libraries(panda_gui Qt5::Core Qt5::Widgets
)

add_executable(panda_joint_positions_bridge src/panda_joint_positions_bridge.cpp)
target_link_libraries(panda_joint_positions_bridge panda_bridge_core)

add_executable(panda_joint_velocities_bridge src/panda_joint_velocities_bridge.cpp)
target_link_libraries(panda_joint_velocities_bridge panda_bridge_core)

add_executable(panda_joint_torques_bridge src/panda_joint_torques_bridge.cpp)
target_link_libraries(panda_joint_torques_bridge panda_bridge_core)

add_executable(panda_cartesian_position_bridge src/panda_cartesian_position_bridge.cpp)
target_link_libraries(panda_cartesian_position_bridge panda_bridge_core)

add_executable(panda_cartesian_velocity_bridge src/panda_cartesian_velocity_bridge.cpp)
target_link_libraries(panda_cartesian_velocity_bridge panda_bridge_core)

add_executable(simple_joints_traj ./src/simple_joint_traj.cpp)
ament_target_dependencies(simple_joints_traj rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs
  rclcpp_lifecycle realtime_tools geometry_msgs)

add_executable(simple_velocity_traj ./src/simple_velocity_joint_traj.cpp)
ament_target_dependencies(simple_velocity_traj rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs
  rclcpp_lifecycle realtime_tools geometry_msgs)

add_executable(simple_torque_cmd src/simple_torque_cmd.cpp)
ament_target_dependencies(simple_torque_cmd rclcpp sensor_msgs realtime_tools)

add_executable(simple_cartesian_traj src/simple_cartesian_traj.cpp)
ament_target_dependencies(simple_cartesian_traj rclcpp geometry_msgs uclv_panda_interfaces realtime_tools tf2_eigen)


install(TARGETS
  panda_joint_velocities_bridge
  panda_joint_positions_bridge
  panda_joint_torques_bridge
  panda_cartesian_velocity_bridge
  panda_cartesian_position_bridge
  simple_joints_traj
  simple_velocity_traj
  simple_torque_cmd
  simple_cartesian_traj
  panda_gui

  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch/
    gui/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  gui/panda_gui_py.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME panda_gui_py
)

ament_export_include_directories(include)
ament_export_dependencies(tf2_eigen)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # The code does not follow the ROS formatting style: the style linters would fail on every file
  # and hide the failures that matter
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_flake8 ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_jitter_buffer test/test_jitter_buffer.cpp)
  target_link_libraries(test_jitter_buffer panda_bridge_core)
  ament_add_gtest(test_joint_tracker test/test_joint_tracker.cpp)
  target_link_libraries(test_joint_tracker panda_bridge_core)
  ament_add_gtest(test_joint_velocity_tracker test/test_joint_velocity_tracker.cpp)
  target_link_libraries(test_joint_velocity_tracker panda_bridge_core)
  ament_add_gtest(test_cartesian_tracker test/test_cartesian_tracker.cpp)
  target_link_libraries(test_cartesian_tracker panda_bridge_core)
  ament_add_gtest(test_cartesian_pipeline test/test_cartesian_pipeline.cpp)
  target_link_libraries(test_cartesian_pipeline panda_bridge_core)
  ament_add_gtest(test_joint_position_pipeline test/test_joint_position_pipeline.cpp)
  target_link_libraries(test_joint_position_pipeline panda_bridge_core)
  ament_add_gtest(test_joint_velocity_pipeline test/test_joint_velocity_pipeline.cpp)
  target_link_libraries(test_joint_velocity_pipeline panda_bridge_core)
  ament_add_gtest(test_joint_torque_pipeline test/test_joint_torque_pipeline.cpp)
  target_link_libraries(test_joint_torque_pipeline panda_bridge_core)
  ament_add_gtest(test_playback_clock test/test_playback_clock.cpp)
  target_link_libraries(test_playback_clock panda_bridge_core)
  # Lifecycle and ROS interfaces of the bridges on the fake robot backend. The test picks its own
  # ROS domain (215-232, localhost only) so that its commands never reach a real bridge
  ament_add_gtest(test_bridges test/test_bridges.cpp TIMEOUT 240)
  target_link_libraries(test_bridges panda_bridge_core)
endif()

ament_package()
