cmake_minimum_required( VERSION 3.20 )
project( joinMap )

set( CMAKE_BUILD_TYPE Release )
# set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( joinMap joinMap.cpp )
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

message("------------------------------------------------------")
message("== OpenCV_LIBS: ${OpenCV_LIBS}")
message("== PCL_LIBRARIES: ${PCL_LIBRARIES}")
message("== PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message("== PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
message("== PCL_VERSION: ${PCL_VERSION}")
message("== PCL_VERSION_MAJOR: ${PCL_VERSION_MAJOR}")
message("== PCL_COMPONENTS: ${PCL_COMPONENTS}")
message("== PCL_DIR: ${PCL_DIR}")
message("------------------------------------------------------")