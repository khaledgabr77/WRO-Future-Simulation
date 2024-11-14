cmake_minimum_required(VERSION 3.5)
project(robot_perception)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(yolov8_msgs REQUIRED)


# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(lidar2image_node src/lidar2image_node.cpp)
add_executable(rgb_to_bgr_converter src/rgb_to_bgr_converter.cpp)

# Link libraries
ament_target_dependencies(lidar2image_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  tf2
  tf2_geometry_msgs
  message_filters
  yolov8_msgs
  vision_msgs
  image_transport
)

target_link_libraries(lidar2image_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS lidar2image_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(rgb_to_bgr_converter
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS rgb_to_bgr_converter
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
