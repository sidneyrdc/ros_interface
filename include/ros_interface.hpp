/**********************************************************************
*	ROS Communication Interface declaration
*	Written by Sidney RDC, 2015.
*	Last Change:2015 Abr 04 14:42:23
**********************************************************************/

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <string>
#include <vector>

// Type of robots supported in the ROS
#define T_REAL 1
#define T_STAGE 2
#define T_TURTLE 3

// Position data type
struct position {
    double x;       // position in x
    double y;       // position in y
    double z;       // position in z
    double roll;    // lateral axis (X)
    double pitch;   // longitudinal axis (Y)
    double yaw;     // vertical axis (Z)
};

// Velocity data type
struct velocity {
    double x;       // velocity in x
    double y;       // velocity in y
    double z;       // velocity in z
    double roll;    // lateral axis angular velocity
    double pitch;   // longitudinal axis angular velocity
    double yaw;     // vertical axis angular velocity
};

// Node class to ROS interface
class ros_interface {
public:
    // Default initializer
    ros_interface();

    // Initialize the ROS communication interface
    ros_interface(int argc, char** argv, std::string node_name);

    // Destructor
    ~ros_interface();

    // Add nodes to the interface
    void add_node(int id, int type, std::string target, position init_pose);

    // A specific node send a message
    void node_send(const int id, velocity msg);

    // A specific node receive a message
    position node_receive(const int id);

    // Data capture frequency (Hz)
    void clock(const float time);

    // Return true if ros_interface is running
    bool ros_ok();
private:
    // Nodes pointer array
    std::vector<void*> nodes_ptr;

    // Verify if the node id exist in nodes array and return its index
    bool check_node(const int id, unsigned int &index);
};

#endif

