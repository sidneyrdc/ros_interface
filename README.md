################################################################################
# ros_interface
# Shared library to ROS integration
# Written by Sidney RDC, 2015.
################################################################################

This source must be compiled on catkin workspace from ROS environment. Once 
compiled, the shared library libros_interface.so and the header ros_interface.hpp 
must be added to your directory, and the lines added to LD_FLAGS in your makefile:
	 -lros_interface 
	 
In your source code, you must add the follow line:
	#include "ros_interface.hpp"


