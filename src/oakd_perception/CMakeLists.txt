cmake_minimum_required(VERSION 3.16)
project(oakd_perception)

option(OAKD_BUILD_CPP_NODE "Build the native C++ OAK-D unified node when DepthAI C++ SDK is available" ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/oakd_depth_node_py
    scripts/oakd_imu_node_py
  DESTINATION lib/${PROJECT_NAME}
)

set(OAKD_CPP_NODE_BUILT FALSE)
if(OAKD_BUILD_CPP_NODE)
  find_package(depthai CONFIG QUIET)

  if(depthai_FOUND)
    add_executable(oakd_unified_node src/oakd_unified_node.cpp)
    target_compile_features(oakd_unified_node PRIVATE cxx_std_17)
    ament_target_dependencies(oakd_unified_node rclcpp sensor_msgs std_msgs)

    if(TARGET depthai::core)
      target_link_libraries(oakd_unified_node depthai::core)
    elseif(TARGET depthai::opencv)
      target_link_libraries(oakd_unified_node depthai::opencv)
    else()
      target_link_libraries(oakd_unified_node depthai)
    endif()

    install(TARGETS oakd_unified_node DESTINATION lib/${PROJECT_NAME})
    set(OAKD_CPP_NODE_BUILT TRUE)
    message(STATUS "oakd_perception: building native C++ oakd_unified_node")
  else()
    message(WARNING
      "oakd_perception: DepthAI C++ SDK was not found; installing the "
      "Python-compatible oakd_unified_node wrapper. Install depthai-core with "
      "CMake config support to enable the native C++ node.")
  endif()
endif()

if(NOT OAKD_CPP_NODE_BUILT)
  install(
    PROGRAMS scripts/oakd_unified_node_py
    DESTINATION lib/${PROJECT_NAME}
    RENAME oakd_unified_node
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
