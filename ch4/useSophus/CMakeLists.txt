cmake_minimum_required(VERSION 3.22)
project(useSophus)

set(CMAKE_BUILD_TYPE Debug)

find_package(Sophus REQUIRED)

include_directories(${SOPHUS_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

add_executable(useSophus useSophus.cpp)
add_executable(useSophus_template useSophus_template.cpp)
add_executable(template_test template_test.cpp)

target_link_libraries(useSophus ${SOPHUS_LIBRARIES})
target_link_libraries(useSophus_template ${SOPHUS_LIBRARIES})

# install(TARGETS useSophus DESTINATION bin)

# install(DIRECTORY ${SOPHUS_INCLUDE_DIRS} DESTINATION include)
# install(DIRECTORY ${SOPHUS_LIBRARY_DIRS} DESTINATION lib)