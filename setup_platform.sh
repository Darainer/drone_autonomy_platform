#!/bin/bash

# This script sets up the environment for the Drone Autonomy Platform.

# Install ROS2
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe
sudo apt-get update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-desktop

# Install colcon
sudo apt-get install -y python3-colcon-common-extensions

# Install dependencies
sudo apt-get install -y ros-humble-mavros ros-humble-mavros-msgs ros-humble-nav2-core
