cmake_minimum_required(VERSION 3.5)
project(gazebo_pg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build C++ executables (if any)
add_executable(robot_controller src/robot_control.cpp)
ament_target_dependencies(robot_controller rclcpp sensor_msgs nav_msgs geometry_msgs)

install(
  DIRECTORY launch materials rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
