# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(ros_interface)

# Find and get all the information about the roscpp package
find_package(roscpp REQUIRED)

# Find Catkin
find_package(catkin REQUIRED)

# Declare this project as a catkin package
catkin_package()

# Add the headers
include_directories(${roscpp_INCLUDE_DIRS}, ${PROJECT_SOURCE_DIR}/include)

# Define a library target called
add_library(ros_interface src/ros_interface.cpp src/node.cpp)

# Define output directory
set_target_properties(ros_interface PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Define compile flags
set_target_properties(ros_interface PROPERTIES COMPILE_FLAGS "-Wall -Wextra -std=c++11 -g")

# Libraries to link
target_link_libraries(ros_interface ${roscpp_LIBRARIES})

