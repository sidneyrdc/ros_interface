# ROS Interface Library

A shared library to simplify the access to the ROS environment. Allows writing
and reading from topics and nodes of ROS without importing ROS libraries and their
dependencies.

## Installation

**NOTE:** The installation procedures are not necessary if you are just a user of the library,
go directly to the usage section ahead.

These library source files are part of a catkin package, and they have to be extracted
into the src folder of your catkin workspace:
```
cd catkin_ws/src
git clone https://github.com/sidneyrdc/ros_interface.git
```

Once it is in your workspace, you can compile the library using the catkin environment:
```
roscd
catkin_make
```

The output is the shared library `libros_interface.so` that can be found in the folder:
```
catkin_ws/src/ros_interface/lib/
```

## Usage

Once you have the library sources compiled, you just need the files `libros_interface.so`
and `ros_interface.hpp` to access the ROS interface.

To use the library in your C/C++ code, put those files in your project's `lib` and
`include` folders, respectively, then add the following line to your code:
```
#include <ros_interface.hpp>
```

Before compiling your project put the following parameter as a compiler option:
```
-lros_interface
```

**NOTE:** Examples of use in C/C++ and Julia can be found in the folder `examples`
of the library source code.
