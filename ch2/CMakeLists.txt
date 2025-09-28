cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE DEBUG)

project(helloSLAM)

add_executable(helloSLAM helloSLAM.cpp)

add_library(hello_shared SHARED libHelloSLAM.cpp)

add_executable(useHello useHello.cpp)
target_link_libraries(useHello hello_shared)
message(STATUS "helloSLAM: ${helloSLAM_SOURCE_DIR}")

# message(WARNING
#     "
#     == 1: ${helloSLAM_SOURCE_DIR}
#     == 2: ${helloSLAM_BINARY_DIR}
#     "
# )

# add_subdirectory(. bin)error
set(EXECUTABLE_OUTPUT_PATH ${helloSLAM_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${helloSLAM_BINARY_DIR}/lib)

install(TARGETS helloSLAM useHello)
