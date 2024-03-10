#define DISABLE_DEBUG
#include "debug.h" // Include the debug.h library to print debug messages

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

/**
 * TODO:
 * 1. [x] Implement map publisher using the grid_map.h library on topic /map
 * 2. [x] Implement a pose publisher using the world_item.h library on topic /robot_pose
 * 3. [x] Implement a tf between the map and the pose of the robot using the tf2_ros library on topic /tf
 * 4. [x] Implement laser scan publisher using the laser_scanner.h library on topic /scan
 * 5. [x] Implement velocity command publisher using the world_item.h library on topic /cmd_vel
 */

/**
 * TOSOLVE:
 * 1. The map is published but it is displayed in the wrong way
 */

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

    ros::init(argc, argv, "map_publisher_node"); // Initialize a ROS node with the name map_publisher_node
    ros::NodeHandle nh;                          // Create a ROS node handle
    Canvas canvas;                               // Create a canvas to draw the world state

    // ROS publishers
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);             // Create a ROS publisher for the map
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1, true);  // Create a ROS publisher for the pose
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1, true);            // Create a ROS publisher for the laser scan
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1, true); // Create a ROS publisher for the velocity command
    tf2_ros::TransformBroadcaster tf_broadcaster;                                                // Create a TF broadcaster

    // Create a GridMap istance and load the map from an image file using the loadFromImage method
    GridMap grid_map(0, 0, 0.1);
    grid_map.loadFromImage("/mnt/c/Users/Emanuele/Desktop/RPT/workspace/catkin_ws/src/ros_2d_multi_robot_simulator/maps/cappero_laser_odom_diag_2020-05-06-16-26-03.png", 0.1); // 0.1 is the resolution of the map

    // Create a Word based on the grid map
    World world_object(grid_map);

    // Create a WorldItem object
    WorldItem object_0(world_object, fromCoefficients(5, 0, 0.5));

    // MAP MESSAGE
    nav_msgs::OccupancyGrid map_msg;                 // Create a message of type OccupancyGrid and set its fields
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

    Vector2f grid_middle(grid_map.cols / 2, grid_map.rows / 2); // Vector that defines the middle of the grid map
    DEBUG_PRINT("Grid middle: " << grid_middle.x() << " " << grid_middle.y());
    DEBUG_PRINT("Grid cols: " << grid_map.cols);
    DEBUG_PRINT("Grid rows: " << grid_map.rows);
    Vector2f world_middle = grid_map.grid2world(grid_middle); // Convert the grid middle to world coordinates

    // Create a UnicyclePlatform object, that is a robot, based on the WorldItem object, and set its pose
    UnicyclePlatform robot(world_object, fromCoefficients(20, -50, 0.785));
    robot.radius = 1;

    ros::Rate loop_rate(10); // Set the ros loop rate to 10Hz
    int count = 0;           // Initialize a counter to know how many times the loop has run

    while (ros::ok())
    {

        map_msg.header.stamp = ros::Time::now(); // Set the timestamp of the map messagee
        map_pub.publish(map_msg);                // Publish the map message

        // Get the current global pose of the robot from the WorldItem object
        Isometry2f robot_pose = robot.pose_in_parent;

        // Create and populate the PoseStamped message
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "pose"; // We set the frame of the pose message to be the robot frame

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
        pose_pub.publish(pose_msg);

        // Create and populate the TF message
        tf2_msgs::TFMessage tf_message;
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "pose";
        transform_stamped.transform.translation.x = robot_pose.translation().x();
        transform_stamped.transform.translation.y = -robot_pose.translation().y();
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_message.transforms.push_back(transform_stamped);

        // Broadcast the TF message
        for (const auto &transform : tf_message.transforms)
        {
            tf_broadcaster.sendTransform(transform);
        }

        // LASER SCAN MESSAGE - This rappresents a single laser scan

        // Create a LaserScan object based on the robot (that is located in a Word item) and set its radius
        LaserScan scan;                                                // This is a single laser scan created using the LaserScan class
        LaserScanner scanner(scan, robot, fromCoefficients(3, 0, -0)); // This is the laser scanner that will be used to scan the environment
        scanner.radius = 0.5;                                          // Set the radius of the laser scanner

        scanner.getScan(); // Get the scan from the laser scanner

        // Create and populate the LaserScan message
        sensor_msgs::LaserScan laser_scan_msg;
        laser_scan_msg.header.stamp = ros::Time::now();
        laser_scan_msg.header.frame_id = "pose"; // We set the frame of the laser solidal to be the robot_pose frame
        laser_scan_msg.angle_min = scan.angle_min;
        laser_scan_msg.angle_max = scan.angle_max;
        laser_scan_msg.angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();
        laser_scan_msg.range_min = scan.range_min;
        laser_scan_msg.range_max = scan.range_max;
        laser_scan_msg.ranges = scan.ranges;
        scan_pub.publish(laser_scan_msg);

        // GEOMETRY MESSAGE - This is the velocity command message

        // Create and populate the TwistStamped message
        geometry_msgs::TwistStamped cmd_vel_msg;
        cmd_vel_msg.header.stamp = ros::Time::now();
        cmd_vel_msg.header.frame_id = "pose";

        float dt = 0.1;
        world_object.tick(dt); // Update the world state
        scanner.tick(dt);      // Update the laser scanner state
        world_object.draw(canvas);
        int ret = showCanvas(canvas, dt * 100);
        if (ret > 0)
            std::cerr << "Key pressed: " << ret << std::endl;
        switch (ret)
        {
        case 81: // left
            std::cout << "Left" << std::endl;
            robot.rv += 0.1;
            cmd_vel_msg.twist.angular.z = robot.rv;
            break;
        case 82: // up
            std::cout << "Up" << std::endl;
            robot.tv += 0.1;
            cmd_vel_msg.twist.linear.x = robot.tv;
            break;
        case 83: // right
            std::cout << "Right" << std::endl;
            robot.rv -= 0.1;
            cmd_vel_msg.twist.angular.z = robot.rv;
            break;
        case 84: // down
            std::cout << "Down" << std::endl;
            robot.tv -= 0.1;
            cmd_vel_msg.twist.linear.x = robot.tv;
            break;
        case 32: // space
            std::cout << "Space" << std::endl;
            robot.rv = 0;
            cmd_vel_msg.twist.angular.z = robot.rv;
            robot.tv = 0;
            cmd_vel_msg.twist.linear.x = robot.tv;
            break;
        default:
            break;
        }

        // Publish cmd_vel message
        cmd_vel_pub.publish(cmd_vel_msg);

        ros::spinOnce();   // Allow the ROS node to process incoming messages
        loop_rate.sleep(); // Sleep for the remaining time to let us run at 10Hz
        ++count;           // Increment the count to know how many times the loop has run
    }

    return 0;
}