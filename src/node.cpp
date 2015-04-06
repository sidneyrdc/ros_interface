/**********************************************************************
*   Node class implementation to ROS interface
*   Written by Sidney RDC, 2015.
*   Last Change:2015 Abr 04 05:08:15
**********************************************************************/

#include "node.hpp"

using namespace std;

// Default constructor
node::node() {
    // NOP
}

// Initialize node interface
node::node(const int id, const int type, const string target, const position init_pose) {
    this->id = id;
    this->type = type;
    this->init_pose = init_pose;
    this->pose = init_pose;
    stringstream topic;

    // Make the subscriber and publisher according with the robot type
    // > turtlesim robots
    if(type == T_TURTLE) {
        // Kill the default turtle
        ros::service::waitForService("kill");
        ros::ServiceClient kill_turtle = nh.serviceClient<turtlesim::Kill>("kill");
        turtlesim::Kill kill;
        kill.request.name = "turtle1";
        kill_turtle.call(kill);

        // Create new turtle
        ros::service::waitForService("spawn");
        ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn spawn;

        // Set initial positions
        spawn.request.x = init_pose.x;
        spawn.request.y = init_pose.y;
        spawn.request.theta = init_pose.yaw;

        // Set name of turtle
        spawn.request.name = target;

        // Add new turtle
        add_turtle.call(spawn);

        // Make the topic with "target/pose"
        topic.str("");
        topic << target << "/pose";

        // Subscribe position
        sub_pose = nh.subscribe(topic.str(),B_SIZE,&node::turtle_callback,this);

    // > stage and real robots
    } else {
        // Clean inputstream
        topic.str("");

        // Make the topic with "target/odom" to stage robots
        if(type == T_STAGE) topic << target << "/odom";

        // Make the topic with "target/pose" to real robots
        else if(type == T_REAL) topic << target << "/pose";

        // Subscribe odometry position
        sub_pose = nh.subscribe(topic.str(),B_SIZE,&node::odom_callback,this);
    }

    // Make the topic with "target/cmd_vel"
    topic.str("");
    topic << target << "/cmd_vel";

    // Publish velocity
    pub_vel = nh.advertise<geometry_msgs::Twist>(topic.str(),B_SIZE);
}

// Destructor
node::~node() {
    // NOP
}

// Publish a velocity message
void node::publish(velocity vel_msg) {
    // Create velocity message package
    geometry_msgs::Twist msg;

    // Fill the message package
    msg.linear.x = vel_msg.x;
    msg.linear.y = vel_msg.y;
    msg.linear.z = vel_msg.z;
    msg.angular.x = vel_msg.roll;
    msg.angular.y = vel_msg.pitch;
    msg.angular.z = vel_msg.yaw;

    // Publish the message
    pub_vel.publish(msg);
}

// Get position
position node::get_pose() {
    return pose;
}

// Get velocity
velocity node::get_vel() {
    return vel;
}

// Turtlesim position callback function
void node::turtle_callback(const turtlesim::Pose::ConstPtr &msg) {
    // Get positions
    pose.x = msg->x;
    pose.y = msg->y;
    pose.yaw = msg->theta;

    // Get velocities
    vel.x = msg->linear_velocity;
    vel.yaw = msg->angular_velocity;
}

// Odometry position callback function
void node::odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Get positions
    pose.x = msg->pose.pose.position.x + init_pose.x;
    pose.y = msg->pose.pose.position.y + init_pose.y;
    pose.z = msg->pose.pose.position.z + init_pose.z;

    // Get Euler angles
    pose.yaw = tf::getYaw(msg->pose.pose.orientation);
}

// Show the position to the node
void node::show_pose() {
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
        << "robot:" << id << " "
        << "position=(" << pose.x << "," << pose.y << ")"
        << " direction=" << pose.yaw);
}

