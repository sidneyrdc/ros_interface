#!/usr/bin/env julia

using Cxx

# importing shared c++ library and header file
const path_to_lib = pwd()
addHeaderDir(path_to_lib, kind=C_System)
Libdl.dlopen(path_to_lib * "/../lib/libros_interface.so", Libdl.RTLD_GLOBAL)
cxxinclude("../include/ros_interface.hpp")

# instantiate a 'ros_interface' object
ros_com = @cxxnew ros_interface(pointer("julia_ros"))
#=ros_com = @cxxnew ros_interface()=#

while  1 == 1
end

