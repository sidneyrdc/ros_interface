#!/usr/bin/env julia

#==============================================================================
 = Example of utilization of the library 'ros_interface'
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2018 Set 17 19:21:48
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

@cxx pose1->set_x(2)
@cxx pose1->set_y(4)

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
const MAX_VX = 1                  # maximum linear velocity
const MAX_VA = 1                  # maximum angular velocity

# dead zones
const MAX_DX = 10                 # maximum linear variation
const MIN_DX = 0.1                # minimum linear variation
const MAX_DA = pi/4               # maximum angular variation
const MIN_DA = 0.01               # minimum angular variation

# variables of proportionality
kx = 0                            # to linear control
ka = 0                            # to angular control

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
    if da > -MIN_DA && da < MIN_DA
        ka = 0
    elseif da > MAX_DA
        ka = 1
    elseif da < -MAX_DA
        ka = -1
    elseif abs(da) >= MIN_DA && abs(da) <= MAX_DA
        ka = da/MAX_DA
    end

    # calculate the angular control signal
    u[2] = MAX_VA*ka

    # linear distance between the reference and the current point
    dx = norm(r[1:2] - x[1:2])

    # calculate the linear control gain
    if dx < MIN_DX
        kx = 0
    elseif dx > MAX_DX
        kx = 1
    elseif dx >= MIN_DX && dx <= MAX_DX
        kx = dx/MAX_DX
    end

    # calculate the linear control signal
    u[1] = MAX_VX*kx*(1 - abs(ka))

    # set the calculated velocities
    @cxx vel1->set_x(u[1])
    @cxx vel1->set_yaw(u[2])

    # send velocities to ROS
    @cxx ros_com->node_vel(1, vel1)

    # print robot's position and control actions
    @printf("x_r:%.2f y_r:%.2f x:%.2f y:%.2f k_x:%.2f u_x:%.2f\n", r[1], r[2], x[1], x[2], kx, u[1])
    @printf("θ_r:%.2f θ:%.2f k_θ:%.2f u_θ:%.2f\n", r[3]*180/pi, x[3]*180/pi, ka, u[2]*180/pi)
    println("time->$(@cxx ros_com->get_time())")
end

