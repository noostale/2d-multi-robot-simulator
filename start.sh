#!/bin/bash
# Run this script to start the 2D multi-robot simulator
# (Using xterm just to show in a old-school way the project logo)

# Ask if they want to start rzviz
echo "Do you want to start rviz? (y/n)"
read start_rviz

if [ $start_rviz == "y" ]; then
    xterm -e 'rviz' &
fi

echo "Do you want to start rqt_graph? (y/n)"
read start_rqt_graph

if [ $start_rqt_graph == "y" ]; then
    xterm -e 'rqt_graph' &
fi

# Start roscore
xterm -e 'roscore' &

# Start the simulator
xterm -fa 'Monospace' -e './devel/lib/ros_2d_multi_robot_simulator/robsim_node' &

