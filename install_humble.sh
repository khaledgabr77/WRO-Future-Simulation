#!/bin/bash

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Set up locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 GPG key using curl directly into the apt keyring
sudo apt install -y curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null

# Add ROS 2 apt repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Update apt and install ROS 2 Humble
sudo apt update

# Install ROS 2 Humble desktop version
sudo apt install ros-humble-desktop -y

# Install additional ROS 2 packages
sudo apt install -y python3-argcomplete \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-vcstool \
  build-essential \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rmw-fastrtps-cpp

# Source ROS 2 setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create a workspace and build it
# mkdir -p ~/ros2_humble_ws/src
# cd ~/ros2_humble_ws/
# colcon build

echo "ROS 2 Humble setup is complete."
