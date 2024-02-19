# Notes about ROS

The first thing to do is create a catkin workspace. This is a folder where all the ROS packages will be stored. To create a catkin workspace, open a terminal and type:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

A workspace is a folder with a particular structure. The src folder is where all the packages are stored. The catkin_make command is used to build the packages in the workspace. The catkin_make command must be run from the root of a catkin workspace. It will look for a `CMakeLists.txt` file in the src folder. This file is used to configure the build process. The CMakeLists.txt file is a standard CMake file. The catkin_make command is a wrapper around the CMake command. It will automatically invoke cmake and other necessary commands to build the packages in the workspace.

After creating the workspace, you must source the generated setup file. This will add the workspace to the `ROS_PACKAGE_PATH` environment variable. This environment variable is used by ROS to find packages. To source the generated setup file, type:

```bash
source devel/setup.bash
```

This command must be run from the root of the catkin workspace. It is recommended to add this command to the end of the `~/.bashrc` file. This will ensure that the workspace is automatically added to the `ROS_PACKAGE_PATH` environment variable whenever a new shell is launched.

To create a new package, use the catkin_create_pkg command. This command must be run from the root of the catkin workspace. The first argument is the name of the package. The `catkin_create_pkg` command will create a package folder with the same name as the package name argument. It will also create a `CMakeLists.txt` file and a package.xml file. The CMakeLists.txt file is used to configure the build process. The package.xml file is used to describe the package.

```bash
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```


After creating a packpage you ha to use the `catkin_make` command to build the packages in the workspace. After building the workspace, you must source the generated setup file. This will add the workspace to the `ROS_PACKAGE_PATH` environment variable. This environment variable is used by ROS to find packages. To source the generated setup file, type:

```bash
source devel/setup.bash
```

So a ROS workspace is the main folder of a ROS project and is made up of several packages. A package is a folder that contains ROS nodes, libraries, datasets, configuration files, etc. A package is the smallest unit of software that can be built and distributed by ROS. A package must contain a package.xml file. This file is used to describe the package. A package must also contain a `CMakeLists.txt` file. This file is used to configure the build process. A package can contain other files and folders. The structure of a package is as follows:

```bash
package_name/
    CMakeLists.txt
    package.xml
    ...
```
