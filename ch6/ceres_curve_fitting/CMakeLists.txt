cmake_minimum_required( VERSION  3.0)
project( ceres_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
# set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)

add_executable( curve_fitting main.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( curve_fitting ${CERES_LIBRARIES} ${OpenCV_LIBS} 
                                    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Charts)

# target_link_libraries(curve_fitting PRIVATE Qt5::Widgets Qt5::Charts)