/******************************************************************************
 * Example of utilization of the library 'ros_interface'
 *
 * Author: Sidney Carvalho - sydney.rdc@gmail.com
 * Last Change: 2017 Oct 23 21:57:06
 * Info: Send and receive information from a node in the ROS environment.
 *****************************************************************************/

#include <iostream>
#include <ros_interface.hpp>

#define PI 3.14159265359

using namespace std;

int main(int argc, char** argv) {
    // instantiate a 'ros_interface' pointer
    ros_interface *ros_com = new ros_interface("ex1");

    // initial positions
    space_t pose1, pose2;

    pose1.x = 0;
    pose1.y = 0;
    pose1.yaw = 0;

    pose2.x = 4;
    pose2.y = 2;
    pose2.yaw = 0;

    // initial velocities
    space_t vel1, vel2;

    // insert the robots (if do you want to use another type of robot, change
    // the second parameter of 'add_node' with the following values:
    // T_REAL for rosaria robots
    // T_STAGE for stageros robots
    // T_TURTLE for turtlesim robots)
    ros_com->add_node(1, T_TURTLE, "robot_0", pose1);
    ros_com->add_node(2, T_TURTLE, "robot_1", pose2);

    // set seed for random number generator
    srand(time(0));

    // main loop
    while(ros_com->ros_ok()) {

        // set the data capture frequency (s)
        ros_com->clock(0.1);

        // set random velocities
        vel1.x = double(rand())/double(RAND_MAX);
        vel1.yaw = 2 * double(rand())/double(RAND_MAX) - 1;

        vel2.x = double(rand())/double(RAND_MAX);
        vel2.yaw = 2 * double(rand())/double(RAND_MAX) - 1;

        // send velocities to ros
        ros_com->node_vel(1, vel1);
        ros_com->node_vel(2, vel2);

        // read positions from ros
        pose1 = ros_com->node_pose(1);
        pose2 = ros_com->node_pose(2);

        // print robot's positions
        cout << "Robot 1 -> x:" << pose1.x << " y:" << pose1.y << " yaw:" << (pose1.yaw * 180 / PI) << endl;
        cout << "Robot 2 -> x:" << pose2.x << " y:" << pose2.y << " yaw:" << (pose1.yaw * 180 / PI) << endl;
    }
}

