#!/bin/bash
# Run this script to install prerequisites in Ubuntu 20.04

# Update the repository list
sudo apt update -y

# Use single-line installer to install ROS-noetic
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh

# Install OpenCV library
sudo apt install -y libopencv-dev

# Install yaml-cpp library
sudo apt install -y libyaml-cpp-dev

# Install Eigen library
sudo apt install -y libeigen3-dev

# install xterm
sudo apt install -y xterm
