cmake_minimum_required(VERSION 3.0)

project(useEigen)

set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)

include_directories(/usr/include/eigen3)

add_executable(eigenMatrix eigenMatrix.cpp)

# target_link_libraries(eigenMatrix ${EIGEN3_LIBRARIES}) 

