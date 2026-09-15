cmake_minimum_required(VERSION 3.8)
project(carmy_motor_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages using the package name so the generated typesupport libraries
# match ROS's expected runtime names.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorMotionControl.msg"
  "msg/MotorFeedback.msg"
  DEPENDENCIES std_msgs
)

# Create a ament index resource to make the package findable
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
)

# Declare a cpp library with a different name to avoid conflict
add_library(${PROJECT_NAME}_lib
  src/motor_cfg.cpp
)

# Rename the output to avoid confusion
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
)

# Use the generated interfaces
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
)

# Motor control test executable. Keep a different target name from the generated
# message target to avoid the CMake name collision while preserving the correct
# ROS typesupport library naming.
add_executable(rs_motor_ros2_node
  src/main.cpp
)

ament_target_dependencies(rs_motor_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
)

rosidl_target_interfaces(rs_motor_ros2_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(rs_motor_ros2_node
  ${PROJECT_NAME}_lib
)

# ament_target_dependencies(enable_motor
#   rclcpp
#   std_msgs
#   can_msgs
# )
# target_link_libraries(enable_motor
#   ${PROJECT_NAME}_lib
# )

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_lib rs_motor_ros2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
