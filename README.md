# Exotica CMake Standalone Example

This example demonstrates how Exotica can be included in a C++ project using CMake and be used without ROS.

## Prerequisites

1. Create a catkin workspace and build Exotica in that workspace
2. Source the workspace

## Building

```
mkdir build
cd build
cmake ..
make -j
```

## Notes

1. We do not use ROS path resolution in the example and instead pass the absolute path to the XML config and URDF / XML via CMake parameters relative to the current source directory at build time. As a consequence, the executables are not relocatable.
