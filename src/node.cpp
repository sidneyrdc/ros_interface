/******************************************************************************
 * Node class to ROS interface <Implementation>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2017 Out 30 20:04:58
 * Info: This file contains the implementation to the node class used by the ROS
 * interface library
 *****************************************************************************/

#include <node.hpp>

using namespace std;

// Default constructor
node::node() {
    // NOP
}

// Initialize node interface
node::node(const int id, const int type, const string target, const space_t init_pose) {
    this->id = id;
    this->type = type;
    this->init_pose = init_pose;
    this->pose = init_pose;
    stringstream topic;

    printf("INFO@libros_interface.so \tAdding node to the interface...\n");

    // Make the subscriber and publisher according with the robot type
    // > turtlesim robots
    if(type == T_TURTLE) {
        // Kill the default turtle
        ros::service::waitForService("kill");
        ros::ServiceClient kill_turtle = nh.serviceClient<turtlesim::Kill>("kill");
        turtlesim::Kill kill;
        kill.request.name = "turtle1";
        kill_turtle.call(kill);

        // Kill the current turtle
        kill.request.name = target;
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

        // Wait for subscriber topic
        while(!check_topic(topic.str())) sleep(2);

        // Subscribe position
        sub_pose = nh.subscribe(topic.str(),B_SIZE,&node::turtle_callback,this);

    // > stage/gazebo and real robots
    } else {
        // Clean inputstream
        topic.str("");

        // Make the topic with "target/odom" to stage robots
        if(type == T_STAGE) topic << target << "/odom";

        // Make the topic with "target/pose" to real robots
        else if(type == T_REAL) topic << target << "/pose";

        // Wait for subscriber topic
        while(!check_topic(topic.str())) sleep(2);

        // Subscribe for Odometry readings
        sub_pose = nh.subscribe(topic.str(), B_SIZE, &node::odom_callback, this);

        // Clean inputstream
        topic.str("");

        // Make the topic with "target/odom" to stage robots
        if(type == T_STAGE) topic << target << "/base_scan_1";

        // Make the topic with "target/pose" to real robots
        else if(type == T_REAL) topic << target << "/base_scan";

        // Subscribe for laser readings
        if(check_topic(topic.str())) sub_laser = nh.subscribe(topic.str(), B_SIZE, &node::laser_callback, this);
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
void node::publish(space_t vel) {
    // Create velocity message package
    geometry_msgs::Twist msg;

    // Fill the message package
    msg.linear.x = vel.x;
    msg.linear.y = vel.y;
    msg.linear.z = vel.z;
    msg.angular.x = vel.roll;
    msg.angular.y = vel.pitch;
    msg.angular.z = vel.yaw;

    // Publish the message
    pub_vel.publish(msg);
}

// Get position
space_t node::get_pose() {
    return pose;
}

// Get velocity
space_t node::get_vel() {
    return vel;
}

// Get laser readings
vector<float> node::get_laser() {
    return laser;
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

// Laser readings callback function
void node::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // Get laser readings
    laser = msg->ranges;
}

// Show the position to the node
void node::show_pose() {
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
        << "robot:" << id << " "
        << "position=(" << pose.x << "," << pose.y << ")"
        << " direction=" << pose.yaw);
}

// Verify if a topic exist
bool check_topic(const string topic) {
    stringstream cmd;

    // Set cmd with "rostopic info topic"
    cmd.str("");
    cmd << "rostopic info " << topic;

    // Test the topic existence
    if(!system(cmd.str().c_str())) return true;

    return false;
}

// Verify if there are publishers in a topic
bool check_pub(const char *topic) {
    stringstream cmd;
    char buffer[50];

    // Set cmd with "rostopic info topic | grep Publishers" (to get the list of
    // publishers in such topic)
    cmd.str("");
    cmd << "rostopic info " << topic << " | grep Publishers";

    // Executes cmd in the shell and store its output in a pointer
    FILE *shell_output = popen(cmd.str().c_str(), "r");

    // If the command fails, return false
    if (!shell_output) return false;

    // Get the execution answer and store it in a buffer
    fgets(buffer, sizeof(buffer), shell_output);

    // Check if the output contains the word "None" or is empty (it is related
    // with the publishers of a topic)
    if(strstr(buffer, "None") != NULL || buffer[0] == '\0') return false;

    // Close output pointer
    pclose(shell_output);

    return true;
}

