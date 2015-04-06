/**********************************************************************
*	ROS Communication Interface implementation
*	Written by Sidney RDC, 2015.
*	Last Change:2015 Abr 04 17:44:58
**********************************************************************/

#include "ros_interface.hpp"
#include "node.hpp"

using namespace std;
using namespace ros;

// Default initializer
ros_interface::ros_interface() {
    // NOP
}

// Initialize the ROS communication interface
ros_interface::ros_interface(int argc, char** argv, string node_name) {
    ros::init(argc,argv,node_name);
}

// Destructor
ros_interface::~ros_interface() {
    // Clear nodes pointer array
    nodes_ptr.clear();
}

// Add nodes to the interface
void ros_interface::add_node(int id, int type, string target, position init_pose) {
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
        node *temp = (node*) nodes_ptr[i];

        if(temp->id == id) {
            index = i;
            return true;
        }
    }

    return false;
}

// A specific node send a message
void ros_interface::node_send(const int id, velocity msg) {
    unsigned int node_index;

    // Publish message if the id node exist
    if(check_node(id,node_index)) {
        node *temp = (node*) nodes_ptr[node_index];
        temp->publish(msg);
    }
}

// A specific node receive a message
position ros_interface::node_receive(const int id) {
    unsigned int node_index;

    // Receive a pose message from the id robot
    if(check_node(id,node_index)) {
        node *temp = (node*) nodes_ptr[node_index];
        return temp->get_pose();
    }

    return position();
}

// Data capture frequency (Hz)
void ros_interface::clock(const float time) {
    // Set loop rate
    ros::Rate rate(time);

    // Time for execute callback functions
    ros::spinOnce();

    // Wait time for another iteration
    rate.sleep();
}

// Return true if ros_interface is running
bool ros_interface::ros_ok() {
    return ros::ok();
}

