#!/usr/bin/env julia

#==============================================================================
 = Example of utilization of the library 'ros_interface'
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2017 Out 25 20:17:31
 = Info: Send and receive information from a node in the ROS environment.
 =============================================================================#

# check if the Cxx package is installed
if typeof(Pkg.installed("Cxx")) == Void
    Pkg.add("Cxx")
end

# package to wrapper C++ code (demands Julia v0.6 or later)
using Cxx

# default C++ lib and include paths
const path_to_lib = "../lib/"
const path_to_include = "../include/"

# define the path_to_include as default C++ headers folder
addHeaderDir(path_to_include, kind=C_System)

# load ROS interface shared library
Libdl.dlopen(path_to_lib * "libros_interface.so", Libdl.RTLD_GLOBAL)

cxxinclude("ros_interface.hpp")                 # ROS interface header
cxxinclude("vector")                            # C++ vector header

# instantiate a 'ros_interface' object
ros_com = @cxxnew ros_interface(pointer("julia_ros"))

# initial positions
pose1 = @cxxnew space_t()
pose2 = @cxxnew space_t()

@cxx pose1->zero()
@cxx pose2->set_x(4)
@cxx pose2->set_y(2)
@cxx pose2->set_yaw(0)

# initial velocities
vel1 = @cxxnew space_t()
vel2 = @cxxnew space_t()

# insert the robots (if do you want to use another type of robot, change
# the second parameter of 'add_node' with the following values - for now, julia
# does not support macros from C, so use its equivalence in integer):
# T_REAL for rosaria robots (use 1)
# T_STAGE for stageros robots (use 2)
# T_TURTLE for turtlesim robots (use 3)
@cxx ros_com->add_node(1, 2, pointer("r1"), pose1)
@cxx ros_com->add_node(2, 2, pointer("r2"), pose2)

# main loop
while @cxx ros_com->ros_ok()
    # set the data capture frequency (s)
    @cxx ros_com->clock(0.1)

    # set random velocities
    @cxx vel1->set_x(Cdouble(2 - randn(1)[1]))
    @cxx vel1->set_yaw(Cdouble(2*randn(1)[1]))

    @cxx vel2->set_x(Cdouble(2 - randn(1)[1]))
    @cxx vel2->set_yaw(Cdouble(2*randn(1)[1]))

    # send velocities to ROS
    @cxx ros_com->node_vel(1, vel1)
    @cxx ros_com->node_vel(2, vel2)

    # read positions from ROS
    pose1 = @cxx ros_com->node_pose(1)
    pose2 = @cxx ros_com->node_pose(2)

    # laser readings from ROS
    lc1 = @cxx ros_com->node_laser(1)
    lc2 = @cxx ros_com->node_laser(2)

    # translate raw C++ readings to Julia array
    laser1 = unsafe_wrap(Array, pointer(lc1), length(lc1))
    laser2 = unsafe_wrap(Array, pointer(lc2), length(lc2))

    println("laser 1 -> n:$(length(laser1)) data:$(laser1)")

    # print robot's positions
    println("Robot 1 -> x:$(@cxx pose1->x) y:$(@cxx pose1->y) yaw:$((@cxx pose1->yaw)*180/pi)")
    println("Robot 2 -> x:$(@cxx pose2->x) y:$(@cxx pose2->y) yaw:$((@cxx pose2->yaw)*180/pi)")
end

