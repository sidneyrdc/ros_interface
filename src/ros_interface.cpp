/******************************************************************************
 * ROS Communication Interface <Implementation>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2017 Nov 13 15:06:23
 * Info: This file contains the implementation to the ROS interface library
 *****************************************************************************/

#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <thread>
#include <ros_interface.hpp>
#include <node.hpp>
#include <signal.h>

using namespace std;

// Node handle and publisher to topic "/clock"
struct clocker {
    ros::NodeHandle nh;
    ros::Publisher pub_clock;
};

// Node handle and publishers to control the simulator (used in v-rep)
struct sim_control {
    ros::NodeHandle nh;
    ros::Publisher sim_start;
    ros::Publisher sim_stop;
};

// Pointer to clocker
clocker *c;

// Is the ROS Interface increasing the ROS clock?
bool inc_clock = false;

// Pointer to sim_control
sim_control *sim;

// Pointer to ros_interface
ros_interface *ros_com;

// Pointer to time thread
std::thread *time_thread;

// Call back to keyboard interruption
void sim_out(int sig) {
    printf("INFO@libros_interface.so \tKeyboard Interruption! Exiting with code %d...\n", sig);

    // Call the destructor of the ros_interface static object
    delete(ros_com);

    // Exit with code sig
    exit(sig);
}

// Increase the time and publish this to topic "/clock"
void inc_time(clocker *c, const float dt) {
    // Time struct from ros
    ros::Time sim_time;

    // Clock message package
    rosgraph_msgs::Clock clock_msg;

    while(ros::ok()) {
        // Get the current time in "/clock"
        sim_time = ros::Time::now();

        // Wait dt time
        usleep(dt/1e-6);

        // Increase simulation time
        sim_time += ros::Duration(dt);

        // Fill the clock message
        clock_msg.clock.sec = sim_time.sec;
        clock_msg.clock.nsec = sim_time.nsec;

        // Publish clock_msg package to topic "/clock"
        c->pub_clock.publish(clock_msg);
    }
}

// Default constructor to space_t type
space_t::space_t() {
    // set all initial coordinates as zero
    zero();
}

// Set all coordinates as zero
void space_t::zero() {
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
}

// Set x coordinate
void space_t::set_x(double val) {
    this->x = val;
}

// Set y coordinate
void space_t::set_y(double val) {
    this->y = val;
}

// Set z coordinate
void space_t::set_z(double val) {
    this->z = val;
}

// Set roll coordinate
void space_t::set_roll(double val) {
    this->roll = val;
}

// Set pitch coordinate
void space_t::set_pitch(double val) {
    this->pitch = val;
}

// Set yaw coordinate
void space_t::set_yaw(double val) {
    this->yaw = val;
}

// Default initializer
ros_interface::ros_interface() {
    // NOP
    printf("INFO@libros_interface \tROS Interface default constructor.\n");
}

// Initialize the ROS communication interface
ros_interface::ros_interface(const char *node_name) {
    // External parameters to ROS initialization function (are not useful)
    char **argv = 0;
    int argc = 0;

    // Store the interface name at ROS environment
    this->node_name = node_name;

    // Associate the static pointer with this ros_interface instance
    ros_com = this;

    printf("INFO@libros_interface.so \tStarting ROS Interface...\n");

    // ROS initialization
    ros::init(argc, argv, node_name);

    // Set the time simulation as in "/clock"
    system("rosparam set /use_sim_time true");

    // Check if there are publishers in the topic "/clock"
    if(!check_pub("/clock")) {
        // The ROS Interface will increase the time in "/clock"
        inc_clock = true;

        // Initialize the pointer to clocker
        c = new clocker;

        // Set publisher on clocker
        c->pub_clock = c->nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
    }

    // Initialize the simulator controller
    sim = new sim_control;

    // Set publishers of the simulation controller
    sim->sim_start = sim->nh.advertise<std_msgs::Bool>("/startSimulation", 1);
    sim->sim_stop = sim->nh.advertise<std_msgs::Bool>("/stopSimulation", 1);

    // Wait for the topics be online
    sleep(1);

    // Start the simulation (v-rep)
    std_msgs::Bool msg;
    msg.data = true;
    sim->sim_start.publish(msg);

    // Signal to keyboard interruption (when ^C is pressed)
    signal(SIGINT, sim_out);
}

// Destructor
ros_interface::~ros_interface() {
    printf("INFO@libros_interface.so \tAccessing the ROS Interface destructor...\n");

    // Call shutdown function
    shutdown();
}

// Add nodes to the interface
void ros_interface::add_node(int id, int type, const char *target, space_t init_pose) {
    unsigned int node_index;

    // Verify if the node has already insered in the array
    if(!check_node(id,node_index)) {
        // Create, initialize and store the node
        nodes_ptr.push_back(new node(id, type, target, init_pose));
    }
}

// Overload of add_node function to accept space_t pointer (demanded by julia)
void ros_interface::add_node(int id, int type, const char *target, const space_t *init_pose) {
    // Call add_node function
    add_node(id, type, target, *init_pose);
}

// Verify if the node id exist in nodes array and return its index
bool ros_interface::check_node(const int id, unsigned int &index) {
    for(unsigned int i = 0; i < nodes_ptr.size(); i++) {
        node temp = *(node*) nodes_ptr[i];

        if(temp.id == id) {
            index = i;
            return true;
        }
    }

    return false;
}

// Set the velocity of a specific node
void ros_interface::node_vel(const int id, space_t msg) {
    unsigned int node_index;

    // Publish message if the id node exist
    if(check_node(id, node_index)) {
        node temp = *(node*) nodes_ptr[node_index];
        temp.publish(msg);
    }
}

// Overload of node_vel function to accept space_t pointer (demanded by julia)
void ros_interface::node_vel(const int id, const space_t *msg) {
    // Call node_vel function
    node_vel(id, *msg);
}

// Get the position of a specific node
space_t ros_interface::node_pose(const int id) {
    unsigned int node_index;

    // Receive a pose message from the id robot
    if(check_node(id, node_index)) {
        node temp = *(node*) nodes_ptr[node_index];
        return temp.get_pose();
    }

    return space_t();
}

// Get the laser readings of a specific node
vector<float> ros_interface::node_laser(const int id) {
    unsigned int node_index;

    // Receive the laser readings from the id robot
    if(check_node(id, node_index)) {
        node temp = *(node*) nodes_ptr[node_index];
        return temp.get_laser();
    }

    return vector<float>();
}

// Data capture frequency
void ros_interface::clock(const float dt) {
    // Create the timer thread
    if(!time_thread && inc_clock) time_thread = new std::thread(inc_time, c, dt);

    // Time for execute callback functions
    ros::spinOnce();

    // Set loop rate
    ros::Rate rate(1/dt);

    // Create and fill a clock message
    //rosgraph_msgs::Clock clock_msg;
    //clock_msg.clock.sec = (unsigned int) floor(sim_time);
    //clock_msg.clock.nsec = (unsigned int) round((sim_time - floor(sim_time)) * 1e9) + 1;

    //// Publish clock_msg package to topic "\clock"
    //c->pub_clock.publish(clock_msg);

    //// Increase simulation time
    //sim_time += dt;

    // Wait time for another iteration
    rate.sleep();
}

// Return true if ros_interface is running
bool ros_interface::ros_ok() {
    return ros::ok();
}

// Remove ros_interface instance and all its nodes
void ros_interface::shutdown() {
    printf("INFO@libros_interface.so \tStopping v-rep simulator...\n");

    // Stop the simulation (v-rep)
    std_msgs::Bool msg;
    msg.data = true;
    sim->sim_stop.publish(msg);

    printf("INFO@libros_interface.so \tRemoving node %s from ROS...\n", node_name);

    // Shutdown ROS Interface
    ros::shutdown();

    // Wait some time to correct finish the ROS Interface
    sleep(1);

    printf("INFO@libros_interface.so \tCleaning ros_interface nodes and structures...\n");

    // Remove all the global pointers and arrays
    nodes_ptr.clear();
    delete(c);
    delete(sim);
}

