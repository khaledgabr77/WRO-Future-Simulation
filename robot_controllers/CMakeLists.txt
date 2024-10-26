cmake_minimum_required(VERSION 3.5)
project(robot_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)  # Add Python interpreter

# Include directories
include_directories(include)

# Add C++ executable
add_executable(ftg_node src/ftg_node.cpp)

# Link libraries and dependencies
ament_target_dependencies(ftg_node
  rclcpp
  sensor_msgs
  ackermann_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

# Install C++ executable
install(TARGETS ftg_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python node
# install(PROGRAMS scripts/ftg_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install Python node
# install(PROGRAMS scripts/local_map_avoidance.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
