#!/bin/bash
# Micro-ROS Agent Setup Script

echo "Creating Workspace..."
mkdir -p ~/microros_ws/src
cd ~/microros_ws

echo "Cloning Micro-ROS Setup..."
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

echo "Installing Dependencies..."
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt install python3-vcstool -y

echo "Building Tools..."
colcon build
source install/local_setup.bash

echo "Creating and Building Agent..."
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

echo "Done! Run the agent with: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"
