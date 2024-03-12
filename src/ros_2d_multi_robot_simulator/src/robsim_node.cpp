#define ENABLE_DEBUG // Enable or disable debug messages
#include "debug.h"   // Include the debug.h library to print debug messages

#include <ros/ros.h> // Include the ROS library

// Inlude the necessary ROS messages
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_msgs/TFMessage.h>

// Include the necessary ft2 libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// Include the roblibs libraries
#include "roblibs/grid_map.h"
#include "roblibs/world_item.h"
#include "roblibs/laser_scanner.h"
#include "roblibs/laser_scan.h"

// Parsing and
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

/**
 * TODO:
 * 1. [x] Implement map publisher using the grid_map.h library on topic /map
 * 2. [x] Implement a pose publisher using the world_item.h library on topic /robot_pose
 * 3. [x] Implement a tf between the map and the pose of the robot using the tf2_ros library on topic /tf
 * 4. [x] Implement laser scan publisher using the laser_scanner.h library on topic /scan
 * 5. [x] Implement velocity command publisher using the world_item.h library on topic /cmd_vel
 * 6. [x] Implement the usage of roslauch to start the node
 * 7. [x] Implement a yaml parser to get the map name from the config file
 * 8. [x] Implement struct to memorize robots configurations
 * 9. [x] Implement robot parse loop
 * 10. [x] Implement laser parse loop
 * 11. [x] Implement velocity parse loop
 * 12. [x] Implement multiple robots in the world
 * 13. [x] Implement a roslaunch alternative using a bash script
 * 14. [x] Add a way to control the robots individually using the keyboard
 * 15. [x] Add rviz choice to the launch file
 * 16. [x] Add rqt_graph choice to the launch file
 * 17. [x] Add the radius of the robot to the robot configuration
 */

/**
 * TOSOLVE:
 * 1. The map is published but it is displayed in the wrong way in rviz
 */

// We can use the following structs to store the configuration of the robot and its sensors extracted from a YAML file

/**
 *   system("xterm -e 'roscore' &");
 *   system(("xterm -e './robsim_node " + mapName + "' &").c_str());
 *   system("xterm -e 'rviz' &");
 */

std::string MapName; // Global variable to store the map name

// Select the robot to move using the number keys and then control it using the arrow keys
int selected_robot;

// Robot configuration struct
struct RobotConfig
{
    std::string id;
    std::string frame_id;
    float start_x;
    float start_y;
    float start_alpha;
    float radius;
    float max_linear_velocity;
    float max_angular_velocity;
    std::vector<struct SensorConfig> devices;
};

// Sensor configuration struct
struct SensorConfig
{
    std::string type;
    std::string frame_id;
    std::string topic;
    int beams;
    float range_min;
    float range_max;
};

// Function to parse the robot configurations from a YAML file, it returns a vector of RobotConfig objects
std::vector<RobotConfig> parseRobotConfigs(const std::string &config_file)
{

    std::vector<RobotConfig> robot_configs; // Vector to store the robots configurations

    // Load the YAML file
    YAML::Node config = YAML::LoadFile(config_file);

    // Parse the map name
    MapName = config["environment"]["map"].as<std::string>();

    // Parse the robot configurations
    for (YAML::const_iterator it = config["robots"].begin(); it != config["robots"].end(); ++it)
    {
        RobotConfig robot_config; // Create a RobotConfig struct to store the robot configuration

        YAML::Node robot_node = *it; // Get the current robot node

        // Populate the robot configuration struct
        robot_config.id = robot_node["id"].as<std::string>();
        robot_config.frame_id = robot_node["frame_id"].as<std::string>();
        robot_config.radius = robot_node["radius"].as<float>();
        robot_config.start_x = robot_node["start_position"]["x"].as<float>();
        robot_config.start_y = robot_node["start_position"]["y"].as<float>();
        robot_config.start_alpha = robot_node["start_position"]["alpha"].as<float>();

        robot_config.max_linear_velocity = robot_node["max_velocity"]["linear"].as<float>();
        robot_config.max_angular_velocity = robot_node["max_velocity"]["angular"].as<float>();

        // Parse the sensor configurations
        for (YAML::const_iterator it_dev = robot_node["devices"].begin(); it_dev != robot_node["devices"].end(); ++it_dev)
        {
            SensorConfig sensor_config; // Create a SensorConfig struct to store the sensor configuration

            YAML::Node device_node = *it_dev; // Get the current sensor node

            // Populate the sensor configuration struct
            sensor_config.type = device_node["type"].as<std::string>();
            sensor_config.frame_id = device_node["frame_id"].as<std::string>();
            sensor_config.topic = device_node["topic"].as<std::string>();
            sensor_config.beams = device_node["beams"].as<int>();
            sensor_config.range_min = device_node["range"]["min"].as<float>();
            sensor_config.range_max = device_node["range"]["max"].as<float>();

            // Add the sensor configuration to the vector of sensors contained in the robot configuration struct
            robot_config.devices.push_back(sensor_config);
        }

        // Add the robot configuration to the vector of robot configurations
        robot_configs.push_back(robot_config);
    }

    return robot_configs;
}

// Coefficients to Isometry2f object
Isometry2f fromCoefficients(float tx, float ty, float alpha)
{
    Isometry2f iso;
    iso.setIdentity();
    iso.translation() << tx, ty;
    iso.linear() = Eigen::Rotation2Df(alpha).matrix();
    return iso;
}

int main(int argc, char **argv)
{
    // std::string configName = argv[1]; // Get the config name from the second input argument
    std::vector<RobotConfig> robotsConfig = parseRobotConfigs("/mnt/c/Users/Emanuele/Documents/GitHub/2d-multi-robot-simulator/src/ros_2d_multi_robot_simulator/src/env1.yaml"); // Use the parse function to populate the configs
    // Print the parsed map name
    std::cout << "Map name: " << MapName << std::endl;

    std::string logo = R"(
    ██████╗  ██████╗ ██████╗ ███████╗██╗███╗   ███╗
    ██╔══██╗██╔═══██╗██╔══██╗██╔════╝██║████╗ ████║
    ██████╔╝██║   ██║██████╔╝███████╗██║██╔████╔██║
    ██╔══██╗██║   ██║██╔══██╗╚════██║██║██║╚██╔╝██║
    ██║  ██║╚██████╔╝██████╔╝███████║██║██║ ╚═╝ ██║
    ╚═╝  ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝╚═╝╚═╝     ╚═╝
                                               v1.0
                                                   )";

    std::cout << "2D Multi Robot Simulator - Emanuele Frasca 1836098" << std::endl;
    std::cout << logo << std::endl;

    ros::init(argc, argv, "robsim"); // Initialize a ROS node with the name map_publisher_node
    ros::NodeHandle nh;              // Create a ROS node handle
    Canvas canvas;                   // Create a canvas to draw the world state

    tf2_ros::TransformBroadcaster tf_broadcaster; // Create a TF broadcaster

    // Create a GridMap istance and load the map from an image file using the loadFromImage method
    GridMap grid_map(0, 0, 0.1);
    std::string mapPath = "/mnt/c/Users/Emanuele/Documents/GitHub/2d-multi-robot-simulator/src/ros_2d_multi_robot_simulator/maps/" + MapName;

    grid_map.loadFromImage(mapPath.c_str(), 0.1); // 0.1 is the resolution of the map

    // Create a Word based on the grid map
    World world_object(grid_map);

    // MAP publisher, we can use just one publisher
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    // MAP MESSAGE
    nav_msgs::OccupancyGrid map_msg;                 // Create a message of type OccupancyGrid
    map_msg.header.stamp = ros::Time::now();         // Set the timestamp of the map messagee
    map_msg.header.frame_id = "map";                 // Set the FRAME_ID of the map message
    map_msg.info.resolution = grid_map.resolution(); // Set the resolution of the map from the GridMap instance
    map_msg.info.width = grid_map.cols;              // Set the width of the map from the GridMap instance
    map_msg.info.height = grid_map.rows;             // Set the height of the map from the GridMap instance

    // Populate the data of the map message
    map_msg.data.resize(grid_map.rows * grid_map.cols);
    for (int row = 0; row < grid_map.rows; ++row)
    {
        for (int col = 0; col < grid_map.cols; ++col)
        {
            uint8_t value = grid_map(row, col);
            if (value < 127)
            {
                map_msg.data[row * grid_map.cols + col] = 100; // Occupied
            }
            else
            {
                map_msg.data[row * grid_map.cols + col] = 0; // Free
            }
        }
    }

    map_pub.publish(map_msg); // Publish the map message

    Vector2f grid_middle(grid_map.cols / 2, grid_map.rows / 2); // Vector that defines the middle of the grid map
    DEBUG_PRINT("Grid middle: " << grid_middle.x() << " " << grid_middle.y());
    DEBUG_PRINT("Grid cols: " << grid_map.cols);
    DEBUG_PRINT("Grid rows: " << grid_map.rows);

    Vector2f world_middle = grid_map.grid2world(grid_middle); // Convert the grid middle to world coordinates

    // Vector to store pointers to the robot instances
    // So this is a vector of pointers to UnicyclePlatform objects
    std::vector<UnicyclePlatform *> robots;

    // Vectors to store the publishers for each robot
    // So these are vectors of publishers
    std::vector<ros::Publisher> pose_pubs, scan_pubs, cmd_vel_pubs;

    // Instantiate robots and their publishers
    // We use the & so we can access the robot_config by reference, so that we don't have to copy it
    for (const RobotConfig &robot_config : robotsConfig)
    {
        // Create robot instance dynamically
        // This defines a pointer to a UnicyclePlatform object
        UnicyclePlatform *robot = new UnicyclePlatform(world_object, fromCoefficients(robot_config.start_x, robot_config.start_y, robot_config.start_alpha)); // Create a new robot in a specified position
        robot->radius = robot_config.radius;                                                                                                                  // Set the radius of the robot from the configuration

        // Add the robot to the vector of pointers to robots
        robots.push_back(robot);

        // Create publishers for this robot
        std::string robot_ns = "robot_" + robot_config.id; // Namespace for the robot
        pose_pubs.push_back(nh.advertise<geometry_msgs::PoseStamped>(robot_ns + "/robot_pose", 1, true));
        scan_pubs.push_back(nh.advertise<sensor_msgs::LaserScan>(robot_ns + "/scan", 1, true));
        cmd_vel_pubs.push_back(nh.advertise<geometry_msgs::TwistStamped>(robot_ns + "/cmd_vel", 1, true));
    }

    ros::Rate loop_rate(10); // Set the ros loop rate to 10Hz
    int count = 0;           // Initialize a counter to know how many times the loop has run

    while (ros::ok())
    {

        // Run a loop for every robot in the robots vector
        for (size_t i = 0; i < robots.size(); ++i)
        {
            auto &robot = robots[i]; // Returns a pointer to the i-th robot

            // Create and populate the PoseStamped message
            Isometry2f robot_pose = robot->pose_in_parent; // Get the current pose of the robot from the WorldItem object

            geometry_msgs::PoseStamped pose_msg;                            // Create a message of type PoseStamped
            pose_msg.header.stamp = ros::Time::now();                       // Set the timestamp of the pose message
            pose_msg.header.frame_id = "robot_" + robotsConfig[i].frame_id; // Set the frame of the pose message

            // Calculate the yaw angle of the robot
            double yaw = std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0));

            // Create an angle axis object based on the yaw angle
            Eigen::AngleAxisd angleAxis(yaw, Eigen::Vector3d::UnitZ());

            // Function to convert the angle axis to a quaternion
            Eigen::Quaterniond q(angleAxis);

            // Set the data of the pose_msg message
            pose_msg.pose.position.x = robot_pose.translation().x();
            pose_msg.pose.position.y = robot_pose.translation().y();
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            // Publish the current pose of the robot
            pose_pubs[i].publish(pose_msg);

            // TF MESSAGES

            // Create and Broadcast TF Message
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = "map";
            transform_stamped.child_frame_id = "robot_" + robotsConfig[i].frame_id;
            transform_stamped.transform.translation.x = robot_pose.translation().x();
            transform_stamped.transform.translation.y = robot_pose.translation().y();
            transform_stamped.transform.translation.z = 0.0; // Assuming 2D, z is 0
            transform_stamped.transform.rotation.x = q.x();
            transform_stamped.transform.rotation.y = q.y();
            transform_stamped.transform.rotation.z = q.z();
            transform_stamped.transform.rotation.w = q.w();

            tf_broadcaster.sendTransform(transform_stamped);

            // LASER SCAN MESSAGE - This rappresents a single laser scan

            // Create a LaserScan object based on the robot (that is located in a Word item) and set its radius
            LaserScan scan;                                                 // This is a single laser scan created using the LaserScan class
            LaserScanner scanner(scan, *robot, fromCoefficients(3, 0, -0)); // This is the laser scanner that will be used to scan the environment
            // Set the parameters of the laser scanner using the values contained in the robotsConfig vector

            scan.range_min = robotsConfig[i].devices[0].range_min; // Set the minimum range of the laser scanner
            scan.range_max = robotsConfig[i].devices[0].range_max; // Set the maximum range of the laser scanner

            scanner.getScan(); // Get the scan from the laser scanner

            sensor_msgs::LaserScan laser_scan_msg;

            laser_scan_msg.header.stamp = ros::Time::now();
            laser_scan_msg.header.frame_id = "robot_" + robotsConfig[i].frame_id;
            laser_scan_msg.angle_min = scan.angle_min;
            laser_scan_msg.angle_max = scan.angle_max;
            laser_scan_msg.angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();
            laser_scan_msg.range_min = scan.range_min;
            laser_scan_msg.range_max = scan.range_max;
            laser_scan_msg.ranges = scan.ranges;
            scan_pubs[i].publish(laser_scan_msg);

            // GEOMETRY MESSAGE - This is the velocity command message

            // Create and populate the TwistStamped message
            geometry_msgs::TwistStamped cmd_vel_msg;
            cmd_vel_msg.header.stamp = ros::Time::now();
            cmd_vel_msg.header.frame_id = "robot_" + robotsConfig[i].frame_id;

            float dt = 0.1;
            world_object.tick(dt); // Update the world state
            scanner.tick(dt);      // Update the laser scanner state
            world_object.draw(canvas);
            int ret = showCanvas(canvas, dt * 100);

            if (ret > 0)
                std::cerr << "Key pressed: " << ret << std::endl; // Print the key pressed
            switch (ret)
            {
            case 49: // 0
                std::cout << "Robot 0 selected" << std::endl;
                selected_robot = 0;
                break;
            case 50: // 1
                std::cout << "Robot 1 selected" << std::endl;
                selected_robot = 1;
                break;
            case 51: // 2
                std::cout << "Robot 2 selected" << std::endl;
                selected_robot = 2;
                break;
            case 52: // 3
                std::cout << "Robot 3 selected" << std::endl;
                selected_robot = 3;
                break;
            case 53: // 4
                std::cout << "Robot 4 selected" << std::endl;
                selected_robot = 4;
                break;
            case 54: // 5
                std::cout << "Robot 5 selected" << std::endl;
                selected_robot = 5;
                break;
            case 55: // 6
                std::cout << "Robot 6 selected" << std::endl;
                selected_robot = 6;
                break;
            case 56: // 7
                std::cout << "Robot 7 selected" << std::endl;
                selected_robot = 7;
                break;
            case 57: // 8
                std::cout << "Robot 8 selected" << std::endl;
                selected_robot = 8;
                break;
            case 58: // 9
                std::cout << "Robot 9 selected" << std::endl;
                selected_robot = 9;
                break;
            case 81: // left, changes the angular velocity of the selected robot
                robots[selected_robot]->rv = 1;
                cmd_vel_msg.twist.angular.z = robots[selected_robot]->rv;
                break;
            case 82: // up, changes the linear velocity of the selected robot
                robots[selected_robot]->tv = 5;
                cmd_vel_msg.twist.linear.x = robots[selected_robot]->tv;
                break;
            case 83: // right, changes the angular velocity of the selected robot
                robots[selected_robot]->rv = -1;
                cmd_vel_msg.twist.angular.z = robots[selected_robot]->rv;
                break;
            case 84: // down, changes the linear velocity of the selected robot
                robots[selected_robot]->tv = -5;
                cmd_vel_msg.twist.linear.x = robots[selected_robot]->tv;
                break;
            case 32: // space, stops the selected robot
                robots[selected_robot]->rv = 0;
                cmd_vel_msg.twist.angular.z = robots[selected_robot]->rv;
                robots[selected_robot]->tv = 0;
                cmd_vel_msg.twist.linear.x = robots[selected_robot]->tv;
                break;
            default:
                break;
            }

            // Publish cmd_vel message
            cmd_vel_pubs[i].publish(cmd_vel_msg);
        }

        ros::spinOnce();   // Allow the ROS node to process incoming messages
        loop_rate.sleep(); // Sleep for the remaining time to let us run at 10Hz
        ++count;           // Increment the count to know how many times the loop has run
    }

    return 0;
}