#!/bin/bash
# Run this script to start the 2D multi-robot simulator
# (Using xterm just to show in a old-school way the project logo)

# Ask if they want to start rzviz
echo "Do you want to start rviz? (y/n)"
read start_rviz

if [ $start_rviz == "y" ]; then
    start_rviz=true
else
    start_rviz=false
fi

# Start roscore
xterm -e 'roscore' &

# Start the simulator
xterm -fa 'Monospace' -e './devel/lib/ros_2d_multi_robot_simulator/robsim_node' &

