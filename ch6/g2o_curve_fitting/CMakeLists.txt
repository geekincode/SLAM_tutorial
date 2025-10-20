cmake_minimum_required( VERSION 3.10 )
project( g2o_curve_fitting )

# 设置C++标准
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

# 查找Qt5组件
find_package( Qt5 COMPONENTS Widgets Charts REQUIRED )

# 添加cmake模块以使用g2o库
# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找G2O
find_package( g2o REQUIRED )
include_directories( 
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

# 查找fmt库
find_package( fmt REQUIRED )

# 添加可执行文件
add_executable( curve_fitting main.cpp )
# 与G2O、OpenCV和Qt链接
target_link_libraries( curve_fitting 
    ${OpenCV_LIBS}
    g2o_core g2o_stuff
    fmt::fmt
    Qt5::Widgets Qt5::Charts
)

# 添加可视化程序
add_executable( visualization visualization.cpp )
target_link_libraries( visualization 
    Qt5::Widgets Qt5::Charts
)

# 启用Qt的MOC、RCC和UIC功能
set_target_properties(curve_fitting PROPERTIES
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON
)

set_target_properties(visualization PROPERTIES
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON
)