# 2D Multi Robot ROS simulator - Frasca Emanuele

## What?

Write a C++ configurable multi robot simlator supporting:

- Grid maps
- Unicycle like mobile robots
- Laser Scanners

The system should read the configuration from a text file specifying the layout of the simulation environment:

- for each device : the frame_id, and the topics. If a device is mounted on another device, the position in w.r.t the parent device.
- For a lidar the number of beams and the max/min range.
- For a robot the maximum velocities.

The output should be integrated in ROS and work with the navigation stack.

## How

## How to compile

To compile the project, you need to have installed the following packages in `Ubuntu 20.04`:
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [OpenCV](https://docs.opencv.org/4.5.2/d7/d9f/tutorial_linux_install.html)


Clone the repository and compile it with the following commands:

```bash
catkin_make
```

If you alread have a catkin workspace, you can clone the repository in the `src` folder of your workspace.

## How to run

## How to test
