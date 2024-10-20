#!/bin/bash

# Update and upgrade the system
sudo apt update && sudo apt upgrade -y

# Set up locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install curl and gnupg
sudo apt install -y curl gnupg lsb-release

# Add the Gazebo GPG key manually
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor | sudo tee /etc/apt/keyrings/gazebo-archive-keyring.gpg > /dev/null

# Add the Gazebo Harmonic apt repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update apt and install Gazebo Harmonic
sudo apt update
sudo apt install -y gz-harmonic

# Install ROS 2 Gazebo ROS packages for integration
sudo apt install -y ros-humble-ros-gz-sim \
  ros-humble-ros-gz-interfaces 
  

# Source ROS 2 setup file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Confirm installation
echo "Gazebo Harmonic and ROS 2 Humble integration setup is complete."
