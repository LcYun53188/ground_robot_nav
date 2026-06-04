cmake_minimum_required(VERSION 3.16)
project(nav_guard)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(visual_odom_guard src/visual_odom_guard.cpp)
target_compile_features(visual_odom_guard PRIVATE cxx_std_17)
ament_target_dependencies(
  visual_odom_guard
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)

install(TARGETS visual_odom_guard DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS scripts/safety_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
