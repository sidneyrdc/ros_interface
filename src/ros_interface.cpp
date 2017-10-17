/******************************************************************************
 * ROS Communication Interface <Implementation>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2017 Oct 17 21:13:03
 * Info: This file contains the implementation to the ROS interface library
 *****************************************************************************/

#include <rosgraph_msgs/Clock.h>
#include <unistd.h>
#include <thread>
#include "ros_interface.hpp"
#include "node.hpp"

using namespace std;

// Node handle and publisher to topic "/clock"
struct clocker {
    ros::NodeHandle nh;
    ros::Publisher pub_clock;
};

// Pointer to clocker
clocker *c;

// Pointer to time thread
std::thread *time_thread;

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

// Default initializer
ros_interface::ros_interface() {
    // NOP
}

// Initialize the ROS communication interface
ros_interface::ros_interface(int argc, char** argv, string node_name) {
    // ROS initialization
    ros::init(argc,argv,node_name);

    // Set the time simulation as in "/clock"
    system("rosparam set /use_sim_time true");

    // Initialize the pointer to clocker
    c = new clocker;

    // Set publisher on clocker
    c->pub_clock = c->nh.advertise<rosgraph_msgs::Clock>("/clock",1);

}

// Destructor
ros_interface::~ros_interface() {
    // Delete all node pointers in nodes_ptr
    for(auto x : nodes_ptr) delete *((node**) x);

    // Delete clock struct pointer
    delete(c);
}

// Add nodes to the interface
void ros_interface::add_node(int id, int type, string target, space_t init_pose) {
    unsigned int node_index;

    // Verify if the node has already insered in the array
    if(!check_node(id,node_index)) {
        // Create, initialize and store the node
        nodes_ptr.push_back(new node(id,type,target,init_pose));
    }
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

// A specific node send a message
void ros_interface::node_send(const int id, space_t msg) {
    unsigned int node_index;

    // Publish message if the id node exist
    if(check_node(id,node_index)) {
        node temp = *(node*) nodes_ptr[node_index];
        temp.publish(msg);
    }
}

// A specific node receive a message
space_t ros_interface::node_receive(const int id) {
    unsigned int node_index;

    // Receive a pose message from the id robot
    if(check_node(id,node_index)) {
        node temp = *(node*) nodes_ptr[node_index];
        return temp.get_pose();
    }

    return space_t();
}

// Data capture frequency
void ros_interface::clock(const float dt) {
    // Create the timer thread
    if(!time_thread) time_thread = new std::thread(inc_time,c,dt);

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

