/******************************************************************************
 * ROS Communication Interface <Header>
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2017 Oct 20 22:21:29
 * Info: This file contains the header to the ROS interface library
 *****************************************************************************/

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <vector>

// Type of robots supported in the ROS
#define T_REAL 1
#define T_STAGE 2
#define T_TURTLE 3

// General spatial coordinate type
class space_t {
public:
    // Default constructor
    space_t();

    // Set all coordinates as zero
    void zero();

    // External settlers (used to set each coordinate in julia)
    void set_x(double val);
    void set_y(double val);
    void set_z(double val);
    void set_roll(double val);
    void set_pitch(double val);
    void set_yaw(double val);

    // General space coordinates
    double x;       // coordinate in x
    double y;       // coordinate in y
    double z;       // coordinate in z
    double roll;    // angle of the lateral axis
    double pitch;   // angle of the longitudinal axis
    double yaw;     // angle of the vertical axis
};

// Node class to ROS interface
class ros_interface {
public:
    // Default initializer
    ros_interface();

    // Initialize the ROS communication interface
    ros_interface(const char *node_name);

    // Destructor
    ~ros_interface();

    // Add nodes to the interface
    void add_node(int id, int type, const char *target, space_t init_pose);

    // Overload of add_node function to accept space_t pointer (demanded by julia)
    void add_node(int id, int type, const char *target, const space_t *init_pose);

    // A specific node send a message
    void node_send(const int id, space_t msg);

    // Overload of node_send function to accept space_t pointer (demanded by julia)
    void node_send(const int id, const space_t *msg);

    // A specific node receive a message
    space_t node_receive(const int id);

    // Data capture frequency
    void clock(const float dt);

    // Return true if ros_interface is running
    bool ros_ok();

private:
    // Nodes pointer array
    std::vector<void*> nodes_ptr;

    // Verify if the node id exist in nodes array and return its index
    bool check_node(const int id, unsigned int &index);
};

#endif

