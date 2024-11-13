cmake_minimum_required(VERSION 3.5)
project(wro_firmware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/car_controller.py
  
  DESTINATION lib/${PROJECT_NAME}
)
find_package(PkgConfig)

pkg_check_modules(SERIAL libserial)

# Add executable
add_executable(car_controller src/car_controller.cpp)

# Link libraries
ament_target_dependencies(car_controller
  rclcpp
  std_msgs
  
)

# Include directories
target_include_directories(
  car_controller PUBLIC "${SERIAL_INCLUDE_DIRS}"
)

target_link_libraries(car_controller "${SERIAL_LDFLAGS}")

install(TARGETS
  car_controller

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files (if any)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
