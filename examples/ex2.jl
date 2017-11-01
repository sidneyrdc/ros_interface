#!/usr/bin/env julia

#==============================================================================
 = Example of utilization of the library 'ros_interface'
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2017 Nov 01 21:00:47
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

# initial velocities
vel1 = @cxxnew space_t()

# insert the robots (if do you want to use another type of robot, change
# the second parameter of 'add_node' with the following values - for now, julia
# does not support macros from C, so use its equivalence in integer):
# T_REAL for rosaria robots (use 1)
# T_STAGE for stageros robots (use 2)
# T_TURTLE for turtlesim robots (use 3)
@cxx ros_com->add_node(1, 2, pointer("r1"), pose1)

# control saturations
max_ux = 1
min_ux = 0
max_ua = 0.1
min_ua = -0.1

# reference (x, y, θ)
r = [3.0, 2.0, 0.0]

# control signals
u = zeros(2)

x = zeros(3)

# main loop
while @cxx ros_com->ros_ok()
    # set the data capture frequency (s)
    @cxx ros_com->clock(0.1)

    # read positions from ROS
    pose1 = @cxx ros_com->node_pose(1)
    x[1] = @cxx pose1->x
    x[2] = @cxx pose1->y
    x[3] = @cxx pose1->yaw

    # hypotenuse (got from current point to reference point vector)
    Hy = norm(r[1:2] - x[1:2])

    # variation in x and y axis (got from current y to reference y)
    Vx = r[1] - x[1]
    Vy = r[2] - x[2]

    # angle of reference
    Hy > 0.1 ? abs(Vx) > 0.001 ? r[3] = acos(Vx/Hy) : r[3] = asin(Vy/Hy) : r[3] = x[3]

    # calculate the control signals using proportional control
    u[2] = r[3] - x[3]
    abs(u[2]) > pi ? u[2] = -sign(u[2])*(2*pi - abs(u[2])) : 0
    abs(u[2]) < 0.01 ? u[1] = Hy : u[1] = 0

    # limit control output to saturation limits
    u[1] > max_ux ? u[1] = max_ux : u[1] < min_ux ? u[1] = min_ux : 0
    u[2] > max_ua ? u[2] = max_ua : u[2] < min_ua ? u[2] = min_ua : 0

    # set random velocities
    @cxx vel1->set_x(0)
    @cxx vel1->set_yaw(u[2])

    # send velocities to ROS
    @cxx ros_com->node_vel(1, vel1)

    # laser readings from ROS
    #=lc1 = @cxx ros_com->node_laser(1)=#

    # translate raw C++ readings to Julia array
    #=laser1 = unsafe_wrap(Array, pointer(lc1), length(lc1))=#

    # print robot's positions
    #=println("Robot 1 -> x:$(@cxx pose1->x) y:$(@cxx pose1->y) yaw:$((@cxx pose1->yaw)*180/pi)  ")=#
    println("x_r: $(r[1]) y_r: $(r[2]) x: $(x[1]) y: $(x[2]) u_x: $(u[1])")
    println("θ_r: $(r[3]*180/pi) θ: $(x[3]*180/pi) u_θ: $(u[2]*180/pi)")
end

