#include "myslam/g2o_types.h"

namespace myslam
{
/**
 * 计算RGBD边的误差
 * 该函数计算观测值与通过位姿变换后的3D点之间的差值
 */
void EdgeProjectXYZRGBD::computeError()
{
    // _vertices
    // 定义位置：g2o库中g2o::BaseBinaryEdge类的成员变量
    // 数据类型：std::vector<g2o::HyperGraph::Vertex*>（顶点指针的向量）
    // 作用：存储与当前边连接的所有顶点
    // 在EdgeProjectXYZRGBD中的具体含义：
    // _vertices[0]：指向g2o::VertexPointXYZ类型的指针，表示3D空间点顶点
    // _vertices[1]：指向g2o::VertexSE3Expmap类型的指针，表示相机位姿顶点（SE(3)李群）

    // 获取顶点：3D点和位姿
    const g2o::VertexPointXYZ* point = static_cast<const g2o::VertexPointXYZ*> ( _vertices[0] );
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[1] );
    // 计算误差：测量值 - 变换后的3D点坐标
    _error = _measurement - pose->estimate().map ( point->estimate() );

    // _measurement
    // 定义位置：g2o库中g2o::BaseBinaryEdge类的成员变量
    // 数据类型：根据具体边类型而定，在EdgeProjectXYZRGBD中为Eigen::Vector3d
    // 作用：存储该边表示的实际观测值
    // 在EdgeProjectXYZRGBD中的具体含义：
    // 表示从RGB-D相机实际测量到的3D点坐标
    // 在误差计算中：_error = _measurement - pose->estimate().map(point->estimate())
    // 即：实际观测值 - 根据当前位姿估计值投影得到的预测值
    // 关键点：_measurement是优化过程中的"真值"参考，优化目标是最小化预测值与测量值之间的差异
}

/**
 * 线性化RGBD边，计算雅可比矩阵
 * 计算误差函数相对于顶点的雅可比矩阵
 */
void EdgeProjectXYZRGBD::linearizeOplus()
{
    // 获取位姿顶点和3D点顶点
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[1] );
    g2o::SE3Quat T ( pose->estimate() );
    g2o::VertexPointXYZ* point = static_cast<g2o::VertexPointXYZ*> ( _vertices[0] );
    // 将3D点通过位姿变换到相机坐标系下
    Eigen::Vector3d xyz = point->estimate();
    Eigen::Vector3d xyz_trans = T.map ( xyz );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    // 设置相对于3D点的雅可比矩阵(3x3矩阵)
    // 由于误差是 measurement - T*point，所以对point的导数为-T
    _jacobianOplusXi(0,0) = -1;
    _jacobianOplusXi(0,1) = 0;
    _jacobianOplusXi(0,2) = 0;
    _jacobianOplusXi(1,0) = 0;
    _jacobianOplusXi(1,1) = -1;
    _jacobianOplusXi(1,2) = 0;
    _jacobianOplusXi(2,0) = 0;
    _jacobianOplusXi(2,1) = 0;
    _jacobianOplusXi(2,2) = -1;

    // 设置相对于位姿的雅可比矩阵(3x6矩阵)
    // 使用Eigen逗号初始化一次性设置整个矩阵
    _jacobianOplusXj << 
        0, -z,  y, -1,  0,  0,
        z,  0, -x,  0, -1,  0,
       -y,  x,  0,  0,  0, -1;
    // // 对旋转部分的导数（平移部分为0）
    // _jacobianOplusXj ( 0,0 ) = 0;
    // _jacobianOplusXj ( 0,1 ) = -z;
    // _jacobianOplusXj ( 0,2 ) = y;
    // // 对平移部分的导数（旋转部分为-I）
    // _jacobianOplusXj ( 0,3 ) = -1;
    // _jacobianOplusXj ( 0,4 ) = 0;
    // _jacobianOplusXj ( 0,5 ) = 0;

    // _jacobianOplusXj ( 1,0 ) = z;
    // _jacobianOplusXj ( 1,1 ) = 0;
    // _jacobianOplusXj ( 1,2 ) = -x;
    // _jacobianOplusXj ( 1,3 ) = 0;
    // _jacobianOplusXj ( 1,4 ) = -1;
    // _jacobianOplusXj ( 1,5 ) = 0;

    // _jacobianOplusXj ( 2,0 ) = -y;
    // _jacobianOplusXj ( 2,1 ) = x;
    // _jacobianOplusXj ( 2,2 ) = 0;
    // _jacobianOplusXj ( 2,3 ) = 0;
    // _jacobianOplusXj ( 2,4 ) = 0;
    // _jacobianOplusXj ( 2,5 ) = -1;
}

/**
 * 计算仅位姿优化的RGBD边的误差
 * 用于仅优化相机位姿，3D点坐标固定
 */
void EdgeProjectXYZRGBDPoseOnly::computeError()
{
    // 获取位姿顶点
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    // 计算误差：测量值 - 变换后的固定3D点坐标
    _error = _measurement - pose->estimate().map ( point_ );
}

/**
 * 线性化仅位姿优化的RGBD边，计算雅可比矩阵
 * 计算误差函数相对于位姿顶点的雅可比矩阵
 */
void EdgeProjectXYZRGBDPoseOnly::linearizeOplus()
{
    // 获取位姿顶点
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    // 将固定的3D点通过位姿变换到相机坐标系下
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    // 设置雅可比矩阵(3x6)，仅相对于位姿
    _jacobianOplusXi <<
        0, -z, y, -1, 0, 0,
        z, 0, -x, 0, -1, 0,
        -y, x, 0, 0, 0, -1;

    // // 对旋转部分的导数
    // _jacobianOplusXi ( 0,0 ) = 0;
    // _jacobianOplusXi ( 0,1 ) = -z;
    // _jacobianOplusXi ( 0,2 ) = y;
    // // 对平移部分的导数
    // _jacobianOplusXi ( 0,3 ) = -1;
    // _jacobianOplusXi ( 0,4 ) = 0;
    // _jacobianOplusXi ( 0,5 ) = 0;

    // _jacobianOplusXi ( 1,0 ) = z;
    // _jacobianOplusXi ( 1,1 ) = 0;
    // _jacobianOplusXi ( 1,2 ) = -x;
    // _jacobianOplusXi ( 1,3 ) = 0;
    // _jacobianOplusXi ( 1,4 ) = -1;
    // _jacobianOplusXi ( 1,5 ) = 0;

    // _jacobianOplusXi ( 2,0 ) = -y;
    // _jacobianOplusXi ( 2,1 ) = x;
    // _jacobianOplusXi ( 2,2 ) = 0;
    // _jacobianOplusXi ( 2,3 ) = 0;
    // _jacobianOplusXi ( 2,4 ) = 0;
    // _jacobianOplusXi ( 2,5 ) = -1;
}

/**
 * 计算从3D点投影到2D像素的仅位姿边的误差
 * 用于单目相机，将3D点投影到图像平面
 */
void EdgeProjectXYZ2UVPoseOnly::computeError()
{
    // 获取位姿顶点
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    // 世界坐标系下的3D点
    Eigen::Vector3d point_world = point_;
    // 将3D点通过位姿变换到相机坐标系下
    Eigen::Vector3d point_camera = pose->estimate().map ( point_world );
    // 获取观测的像素坐标
    Eigen::Vector2d obs ( _measurement );
    // 计算误差：观测像素值 - 3D点投影到像素的坐标
    // 使用camera2pixel而不是world2pixel避免类型转换问题
    _error = obs - camera_->camera2pixel( point_camera );
}

/**
 * 线性化从3D点投影到2D像素的仅位姿边，计算雅可比矩阵
 * 计算误差函数相对于位姿顶点的雅可比矩阵
 */
void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
    // 获取位姿顶点
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    // 将固定的3D点通过位姿变换到相机坐标系下
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    // 设置雅可比矩阵(2x6)，仅相对于位姿
    // 第一行对应于u坐标的导数
    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 * camera_->fx_;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) * camera_->fx_;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx_;

    // 第二行对应于v坐标的导数
    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) * camera_->fy_;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 * camera_->fy_;
    _jacobianOplusXi ( 1,2 ) = -x/z * camera_->fy_;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z * camera_->fy_;
    _jacobianOplusXi ( 1,5 ) = y/z_2 * camera_->fy_;
}

}