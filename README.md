# 2D Multi Robot ROS simulator - Frasca Emanuele

## What?

The aim of this project is to create a 2D multi-robot simulator using the roblibs and integrating it with ROS. In particular the simulator is able to parse a configuration file and create a simulation environment with multiple robots, devices and a given map.

## How to compile

To compile the project, you need to have installed the following packages in `Ubuntu 20.04`:
- ROS Noetic
- OpenCV
- Eigen
- YAML-CPP

To install the requirements, you can run the following script in the root directory of the project:

```bash
./install_requirements.sh
```

To compile the project, you can run the following script in the root directory of the project:

```bash
./compile.sh
```

This script will add the the workspace to the ROS environment and compile the project.

## How to run

To run the project, you can run the following script in the root directory of the project:

```bash
./run.sh
```

## How to test

You can change the configuration of the simulation by modifying the `config.yaml` file in the `config` directory. The file is self-explanatory and contains the configuration of the simulation environment.
