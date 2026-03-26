################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(dynamixel_workbench_operators)

## Compile as C++17, supported in ROS2 Humble and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find external packages and dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

################################################################################
# Build
################################################################################
add_executable(joint_operator
  src/joint_operator.cpp
)
add_executable(wheel_operator
  src/wheel_operator.cpp
)
# Link non-ROS dependencies
target_link_libraries(joint_operator
  yaml-cpp
)
# Link ROS 2 and Dynamixel dependencies
ament_target_dependencies(joint_operator
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)
ament_target_dependencies(wheel_operator
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# Include directories
target_include_directories(joint_operator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(wheel_operator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

#################################################################################
## Install
#################################################################################
install(TARGETS
  joint_operator
  wheel_operator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config # launch
  DESTINATION share/${PROJECT_NAME}
)
################################################################################
# Test
################################################################################

################################################################################
# ament_packages
################################################################################
ament_export_dependencies(
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

ament_package()