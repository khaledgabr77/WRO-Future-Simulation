cmake_minimum_required(VERSION 3.5)
project(robot_system)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Example dependency
find_package(std_msgs REQUIRED)

# Add your source files and executable (if applicable)
# add_executable(robot_node src/robot_node.cpp)  # Example node file
# target_link_libraries(robot_node rclcpp std_msgs)
# ament_target_dependencies(robot_node rclcpp std_msgs)

# Install executables and other files
install(TARGETS
  # robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and URDF files (if applicable)
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# Finalize the ament package
ament_package()
