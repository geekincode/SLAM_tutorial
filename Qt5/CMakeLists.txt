cmake_minimum_required(VERSION 3.10)
project(Qt5ChartDemo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)

add_executable(Qt5ChartDemo main.cpp)
target_link_libraries(Qt5ChartDemo PRIVATE Qt5::Widgets Qt5::Charts)