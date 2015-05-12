/**********************************************************************
*	Node class declaration to ROS interface
*	Written by Sidney RDC, 2015.
*	Last Change:2015 Abr 11 15:50:20
**********************************************************************/

#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include "ros_interface.hpp"

// Message buffer size
#define B_SIZE 1

// Node class to ROS interface
class node {
public:
    // Default constructor
    node();

    // Initialize node interface
    node(const int id, const int type, const std::string target, const position init_pose);

    // Destructor
    ~node();

    // Unique identifier
    int id;

    // Publish a velocity message
    void publish(velocity vel_msg);

    // Get position
    position get_pose();

    // Get velocity
    velocity get_vel();

    // Show the position to the node
    void show_pose();

private:
    // Type of node (real,stage,turtlesim)
    int type;

    // Type of subscriber message
    position pose;

    // Initial position to node
    position init_pose;

    // Type of publisher message
    velocity vel;

    // Interface node handle
    ros::NodeHandle nh;

    // Publisher in velocity topics
    ros::Publisher pub_vel;

    // Subscriber position topics
    ros::Subscriber sub_pose;

    // Verify if a topic exist
    bool check_topic(const std::string topic);

    // Turtlesim position callback function
    void turtle_callback(const turtlesim::Pose::ConstPtr &msg);

    // Odometry position callback function
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif

