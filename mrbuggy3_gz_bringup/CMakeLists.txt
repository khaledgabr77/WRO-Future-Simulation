cmake_minimum_required(VERSION 3.8)
project(mrbuggy3_gz_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install project RViz files
install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python nodes
install(PROGRAMS
  scripts/odom_to_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
