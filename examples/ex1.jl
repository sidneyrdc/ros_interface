#!/usr/bin/env julia

# check if the Cxx package is installed
if typeof(Pkg.installed("Cxx")) == Void
    Pkg.add("Cxx")
end

using Cxx               # package to wrapper C++ code (demands Julia v0.6 or later)

# importing shared c++ library and header file
const path_to_lib = "../lib/"
addHeaderDir(path_to_lib, kind=C_System)
Libdl.dlopen(path_to_lib * "libros_interface.so", Libdl.RTLD_GLOBAL)
cxxinclude("../include/ros_interface.hpp")

# instantiate a 'ros_interface' object
ros_com = @cxxnew ros_interface(pointer("julia_ros"))

# initial positions
pose1 = @cxxnew space_t()

#=@cxx pose1->x = Cdouble(2)=#
#=x = @cxx pose1->x=#
#=unsafe_store!(x::Ptr{Void}, pointer("a"), 1::Integer)=#

#=@cxx pose1->set_x(Cdouble(2))=#
@cxx pose1->set(Cdouble(3), @cxx pose1->x)

println(@cxx pose1->x)

@cxx ros_com->add_node(Cint(1), Cint(1), pointer("robot_1"), pose1);

#=type space_t=#
    #=x::Cdouble=#
    #=y::Cdouble=#
    #=z::Cdouble=#
    #=row::Cdouble=#
    #=pitch::Cdouble=#
    #=yaw::Cdouble=#
#=end=#

while  1 == 1
end

