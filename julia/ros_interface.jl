#==============================================================================
 = Julia Wrapper to the ros_interface.h Header
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2020 Abr 20 17:20:23
 = Info: Maps all functions of ros_interface into native julia code.
 =============================================================================#

module ROS_INTERFACE

using Cxx           # package to wrapper C++ code (demands Julia v0.6 or later)

# default C++ lib and include paths
global const path_to_lib = "../lib/"
global const path_to_include = "../include/"

# define the path_to_include as default C++ headers folder
addHeaderDir(path_to_include, kind=C_System)

# load ROS interface shared library
Libdl.dlopen(path_to_lib * "libros_interface.so", Libdl.RTLD_GLOBAL)

# include cpp headers
cxxinclude("ros_interface.hpp")                 # ROS interface header
cxxinclude("vector")                            # C++ vector header

# public functions
export ros_interface, add_bot, set_vel, get_pose, get_laser, clock, ros_ok, shutdown, get_time

# initialize ROS communication interface
function ros_interface(node_name::String)
    # instantiate a 'ros_interface' object
    global ros_com = @cxxnew ros_interface(pointer(node_name))
end

# add robots to the interface
function add_bot(bot_id::Int64, bot_type::Int64, bot_name::String, bot_pose::Array{Float64, 1})
    # position array of ros_interface
    ros_pose = @cxxnew space_t()

    # set robot initial position at ROS
    @cxx ros_pose->set_x(bot_pose[1])
    @cxx ros_pose->set_y(bot_pose[2])
    @cxx ros_pose->set_yaw(bot_pose[3])

    # add the robot to the ROS environment
    @cxx ros_com->add_node(bot_id, bot_type, pointer(bot_name), ros_pose)

end

# set the velocity of a specific node
function set_vel(bot_id::Int64, bot_ctrl::Array{Float64, 1})
    # control array of ros_interface
    ros_vel = @cxxnew space_t()

    # translate julia array to ros_interface datatype
    @cxx ros_vel->set_x(bot_ctrl[1])
    @cxx ros_vel->set_y(bot_ctrl[2])
    @cxx ros_vel->set_yaw(bot_ctrl[3])

    # send control actions to ROS
    @cxx ros_com->node_vel(bot_id, ros_vel)

end

# get the position of a specific node
function get_pose(bot_id::Int64)
    # get robot position from ROS
    ros_pose = @cxx ros_com->node_pose(bot_id)

    # translate data from type on ros_interface to julia
    return [@cxx ros_pose->x; @cxx ros_pose->y; @cxx ros_pose->yaw]
end

# get the laser readings of a specific node
function get_laser()
    # TODO: finish this
end

# set data sampling time (seconds) and execute callback functions (must be included in the main loop)
function clock(dt::Float64)
    @cxx ros_com->clock(dt)
end

# check if ros is running
function ros_ok()
    return @cxx ros_com->ros_ok()
end

# remove the instance of ros_interface and all its nodes
function shutdown()
    @cxx ros_com->shutdown()
end

# get ROS current time (seconds)
function get_time()
    return @cxx ros_com->get_time()
end

end

