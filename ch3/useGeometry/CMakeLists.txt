cmake_minimum_required(VERSION 3.0)

project(useGeometry)

set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)

include_directories(/usr/include/eigen3)

add_executable(useGeometry useGeometry.cpp)
add_executable(test_5 test_5.cpp)

