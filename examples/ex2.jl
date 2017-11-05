#!/usr/bin/env julia

#==============================================================================
 = Example of utilization of the library 'ros_interface'
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2017 Nov 05 00:52:33
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
max_vx = 1                  # maximum linear velocity
max_va = 0.1                # maximum angular velocity

# variation limits
max_dx = 10                 # maximum linear variation
min_dx = 0.1                # minimum linear variation
max_da = pi/4               # maximum angular variation
min_da = 0.01               # minimum angular variation

# variables of proportionality
kx = 0                      # to linear control
ka = 0                      # to angular control

# reference (x, y, θ)
r = [3.0, 2.0, 0.0]

# control signals (vx, vθ)
u = zeros(2)

# pose
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

    # sides of triangle formed by current point and reference
    op = r[2] - x[2]                            # opposite side
    ad = r[1] - x[1]                            # adjacent side

    # get the reference angle (the angle that the line initiating in the current
    # point and finishing in the reference gets with the frame x axis)
    # check the quadrants and hold the reference angle between (-π, π)
    if ad >= 0 && op >= 0                       # 1st quadrant
        r[3] = atan(op/ad)
    elseif ad <= 0 && op >= 0                   # 2nd quadrant
        r[3] = pi - atan(abs(op/ad))
    elseif ad <= 0 && op <= 0                   # 3rd quadrant
        r[3] = atan(abs(op/ad)) - pi
    elseif ad >= 0 && op <= 0                   # 4th quadrant
        r[3] = atan(op/ad)
    end

    # calculate the angular variation
    da = r[3] - x[3]

    # get the shorter arc and with opposite signal (to perform the best movement)
    da > pi ? da = da - 2*pi : da < -pi ? da = 2*pi - abs(da) : 0

    # calculate the angular control gain
    if da > -min_da && da < min_da
        ka = 0
    elseif da > max_da
        ka = 1
    elseif da < -max_da
        ka = -1
    elseif abs(da) >= min_da && abs(da) <= max_da
        ka = da/max_da
    end

    # calculate the angular control signal
    u[2] = max_va*ka

    # linear distance between the reference and the current point
    dx = norm(r[1:2] - x[1:2])

    # calculate the linear control gain
    if dx < min_dx
        kx = 0
    elseif dx > max_dx
        kx = 1
    elseif dx >= min_dx && dx <= max_dx
        kx = dx/max_dx
    end

    # calculate the linear control signal
    ka == 0 ? u[1] = max_vx*kx : u[1] = 0

    # set random velocities
    @cxx vel1->set_x(u[1])
    @cxx vel1->set_yaw(u[2])

    # send velocities to ROS
    @cxx ros_com->node_vel(1, vel1)

    # print robot's position and control actions
    @printf("x_r:%.2f y_r:%.2f x:%.2f y:%.2f k_x:%.2f u_x:%.2f\n", r[1], r[2], x[1], x[2], kx, u[1])
    @printf("θ_r:%.2f θ:%.2f k_θ:%.2f u_θ:%.2f\n", r[3]*180/pi, x[3]*180/pi, ka, u[2]*180/pi)
end

