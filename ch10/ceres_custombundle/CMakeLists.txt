cmake_minimum_required(VERSION 3.10)

project(ceres_customBundle)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")


include_directories(${CERES_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIR}
                    /usr/local/include/eigen3
                    ${PROJECT_SOURCE_DIR}/common
                    ${PROJECT_SOURCE_DIR}/common/tools 
                    ${PROJECT_SOURCE_DIR}/common/flags)

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

add_executable(${PROJECT_NAME} ceresBundle.cpp)

target_link_libraries(${PROJECT_NAME} BALProblem ParseCmd ${CERES_LIBRARIES} )