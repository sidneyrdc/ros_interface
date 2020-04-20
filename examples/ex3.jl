#!/usr/bin/env julia

#==============================================================================
 = Example of utilization of the library 'ros_interface'
 =
 = Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
 = Last Change: 2020 Abr 20 17:32:12
 = Info: Send and receive information from a node in the ROS environment.
 =============================================================================#

# add ros_interface wrapper to ros_interface.hpp
include("../julia/ros_interface.jl")

# to call ros_interface functions
using ROS_INTERFACE

# start ROS interface
ros_interface("exe3")

# insert the robots (if do you want to use another type of robot, change
# the second parameter of 'add_node' with the following values - for now, julia
# does not support macros from C, so use its equivalence in integer):
# T_REAL for rosaria robots (use 1)
# T_STAGE for stageros robots (use 2)
# T_TURTLE for turtlesim robots (use 3)
add_bot(1, 2, "robot_1", [0.0; 0.0; 0.0])

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
while ros_ok()
    # set the data capture frequency (s)
    clock(0.1)

    # read positions from ROS
    x = get_pose(1)

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

    # send velocities to ROS
    set_vel(1, [u[1]; 0.0; u[2]])

    # print robot's position and control actions
    @printf("x_r:%.2f y_r:%.2f x:%.2f y:%.2f k_x:%.2f u_x:%.2f\n", r[1], r[2], x[1], x[2], kx, u[1])
    @printf("θ_r:%.2f θ:%.2f k_θ:%.2f u_θ:%.2f\n", r[3]*180/pi, x[3]*180/pi, ka, u[2]*180/pi)
    println("time->$(get_time())")
end

