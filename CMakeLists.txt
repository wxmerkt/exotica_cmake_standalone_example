cmake_minimum_required(VERSION 3.0.2)
project(exotica_cmake_standalone_example)

# Note, this assumes that a catkin workspace containing exotica has been built 
# and is sourced. We will use regular CMake in this file to find the required libraries

find_package(exotica_core REQUIRED)

add_executable(ik_minimal src/ik_minimal.cpp)
target_include_directories(ik_minimal PRIVATE ${exotica_core_INCLUDE_DIRS})
target_compile_definitions(ik_minimal PRIVATE -DEXOTICA_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(ik_minimal PUBLIC ${exotica_core_LIBRARIES})
