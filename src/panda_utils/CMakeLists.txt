cmake_minimum_required(VERSION 3.8)
project(panda_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uclv_panda_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Python support for GUI
find_package(ament_cmake_python REQUIRED)

#Include Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  include
  /opt/ros/jazzy/include
  # ~/libfranka/include/
)

add_executable(panda_gui  src/panda_gui.cpp)
ament_target_dependencies(panda_gui rclcpp rclcpp_lifecycle std_srvs geometry_msgs sensor_msgs)
target_link_libraries(panda_gui Qt5::Core Qt5::Widgets
)

add_executable(panda_joint_velocities_bridge ./src/panda_joint_velocities_bridge.cpp ./src/constants.cpp)
ament_target_dependencies(panda_joint_velocities_bridge rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs rclcpp_lifecycle realtime_tools geometry_msgs)
target_link_libraries(panda_joint_velocities_bridge franka)

add_executable(panda_joint_positions_bridge ./src/panda_joint_positions_bridge.cpp ./src/constants.cpp)
ament_target_dependencies(panda_joint_positions_bridge rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs rclcpp_lifecycle realtime_tools geometry_msgs)
target_link_libraries(panda_joint_positions_bridge franka)

add_executable(panda_cartesian_velocity_bridge ./src/panda_cartesian_velocity_bridge.cpp ./src/constants.cpp)
ament_target_dependencies(panda_cartesian_velocity_bridge rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs rclcpp_lifecycle realtime_tools geometry_msgs)
target_link_libraries(panda_cartesian_velocity_bridge franka)

add_executable(panda_cartesian_position_bridge ./src/panda_cartesian_position_bridge.cpp ./src/constants.cpp)
ament_target_dependencies(panda_cartesian_position_bridge rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs rclcpp_lifecycle realtime_tools geometry_msgs)
target_link_libraries(panda_cartesian_position_bridge franka)

add_executable(simple_joints_traj ./src/simple_joint_traj.cpp)
ament_target_dependencies(simple_joints_traj rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs rclcpp_lifecycle realtime_tools geometry_msgs)
target_link_libraries(simple_joints_traj franka)

add_executable(simple_velocity_traj ./src/simple_velocity_joint_traj.cpp)
ament_target_dependencies(simple_velocity_traj rclcpp uclv_panda_interfaces std_msgs tf2_eigen sensor_msgs rclcpp_lifecycle realtime_tools geometry_msgs)
target_link_libraries(simple_velocity_traj franka)


install(TARGETS
  panda_joint_velocities_bridge
  panda_joint_positions_bridge
  panda_cartesian_velocity_bridge
  panda_cartesian_position_bridge
  simple_joints_traj
  simple_velocity_traj
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
