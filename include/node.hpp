/******************************************************************************
 * Node class to ROS interface <Header>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2017 Out 30 18:13:27
 * Info: This file contains the header to the node class used by the ROS
 * interface library
 *****************************************************************************/

#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <ros_interface.hpp>

// Message buffer size
#define B_SIZE 1

// Node class to ROS interface
class node {
public:
    // Default constructor
    node();

    // Initialize node interface
    node(const int id, const int type, const std::string target, const space_t init_pose);

    // Destructor
    ~node();

    // Unique identifier
    int id;

    // Publish a velocity message
    void publish(space_t vel);

    // Get position
    space_t get_pose();

    // Get velocity
    space_t get_vel();

    // Get laser readings
    std::vector<float> get_laser();

    // Show the position to the node
    void show_pose();

private:
    // Type of node (real,stage,turtlesim)
    int type;

    // Laser scan readings
    std::vector<float> laser;

    // Type of subscriber message
    space_t pose;

    // Initial position to node
    space_t init_pose;

    // Type of publisher message
    space_t vel;

    // Interface node handle
    ros::NodeHandle nh;

    // Publisher in velocity topics
    ros::Publisher pub_vel;

    // Subscriber position topics
    ros::Subscriber sub_pose;

    // Subscriber laser topics
    ros::Subscriber sub_laser;

    // Subscriber sonar topics
    ros::Subscriber sub_sonar;

    // Verify if a topic exist
    bool check_topic(const std::string topic);

    // Turtlesim position callback function
    void turtle_callback(const turtlesim::Pose::ConstPtr &msg);

    // Odometry position callback function
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

    // Laser readings callback function
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
};

#endif

