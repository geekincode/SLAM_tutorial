/**
 * @file g2o_types.cpp
 * @brief g2o自定义边类型的实现
 * 
 * 实现了三种用于视觉里程计优化的边：
 * 1. EdgeProjectXYZRGBD: 3D-3D边（ICP）
 * 2. EdgeProjectXYZRGBDPoseOnly: 仅位姿优化的3D边
 * 3. EdgeProjectXYZ2UVPoseOnly: 3D-2D投影边（PnP）
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#include "myslam/g2o_types.h"

namespace myslam
{

/**
 * @brief EdgeProjectXYZRGBD的误差计算
 * 
 * 误差 = 测量值(观测3D点) - 位姿变换后的3D点
 * e = z - T * p
 * 
 * 用于3D-3D的ICP问题，同时优化位姿和3D点
 */
void EdgeProjectXYZRGBD::computeError()
{
    // 获取顶点：3D点和位姿
    const g2o::VertexPointXYZ* point = static_cast<const g2o::VertexPointXYZ*> ( _vertices[0] );
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[1] );
    
    // 计算误差：测量值 - 变换后的点
    _error = _measurement - pose->estimate().map ( point->estimate() );
}

/**
 * @brief EdgeProjectXYZRGBD的雅可比矩阵计算
 * 
 * 解析求导，分别计算对3D点和位姿的雅可比矩阵
 * 
 * 对3D点的雅可比: de/dp = -R
 * 对位姿的雅可比: de/d(xi) (李代数形式)
 */
void EdgeProjectXYZRGBD::linearizeOplus()
{
    // 获取位姿顶点和估计值
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[1] );
    g2o::SE3Quat T ( pose->estimate() );
    
    // 获取3D点并变换到相机坐标系
    g2o::VertexPointXYZ* point = static_cast<g2o::VertexPointXYZ*> ( _vertices[0] );
    Eigen::Vector3d xyz = point->estimate();
    Eigen::Vector3d xyz_trans = T.map ( xyz );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    // 对3D点的雅可比矩阵: de/dp = -R
    _jacobianOplusXi = - T.rotation().toRotationMatrix();

    // 对位姿的雅可比矩阵 (6维李代数)
    // 对应顺序: [wx, wy, wz, tx, ty, tz] (旋转在前，平移在后)
    _jacobianOplusXj ( 0,0 ) = 0;
    _jacobianOplusXj ( 0,1 ) = -z;
    _jacobianOplusXj ( 0,2 ) = y;
    _jacobianOplusXj ( 0,3 ) = -1;
    _jacobianOplusXj ( 0,4 ) = 0;
    _jacobianOplusXj ( 0,5 ) = 0;

    _jacobianOplusXj ( 1,0 ) = z;
    _jacobianOplusXj ( 1,1 ) = 0;
    _jacobianOplusXj ( 1,2 ) = -x;
    _jacobianOplusXj ( 1,3 ) = 0;
    _jacobianOplusXj ( 1,4 ) = -1;
    _jacobianOplusXj ( 1,5 ) = 0;

    _jacobianOplusXj ( 2,0 ) = -y;
    _jacobianOplusXj ( 2,1 ) = x;
    _jacobianOplusXj ( 2,2 ) = 0;
    _jacobianOplusXj ( 2,3 ) = 0;
    _jacobianOplusXj ( 2,4 ) = 0;
    _jacobianOplusXj ( 2,5 ) = -1;
}

/**
 * @brief EdgeProjectXYZRGBDPoseOnly的误差计算
 * 
 * 仅优化位姿，3D点作为固定参数
 * 误差 = 测量值 - T * point
 */
void EdgeProjectXYZRGBDPoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    _error = _measurement - pose->estimate().map ( point_ );
}

/**
 * @brief EdgeProjectXYZRGBDPoseOnly的雅可比矩阵计算
 * 
 * 仅对位姿求导，与EdgeProjectXYZRGBD的位姿雅可比相同
 */
void EdgeProjectXYZRGBDPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    // 对位姿的雅可比矩阵
    _jacobianOplusXi ( 0,0 ) = 0;
    _jacobianOplusXi ( 0,1 ) = -z;
    _jacobianOplusXi ( 0,2 ) = y;
    _jacobianOplusXi ( 0,3 ) = -1;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = 0;

    _jacobianOplusXi ( 1,0 ) = z;
    _jacobianOplusXi ( 1,1 ) = 0;
    _jacobianOplusXi ( 1,2 ) = -x;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1;
    _jacobianOplusXi ( 1,5 ) = 0;

    _jacobianOplusXi ( 2,0 ) = -y;
    _jacobianOplusXi ( 2,1 ) = x;
    _jacobianOplusXi ( 2,2 ) = 0;
    _jacobianOplusXi ( 2,3 ) = 0;
    _jacobianOplusXi ( 2,4 ) = 0;
    _jacobianOplusXi ( 2,5 ) = -1;
}

/**
 * @brief EdgeProjectXYZ2UVPoseOnly的误差计算
 * 
 * 3D点到2D像素的投影误差，用于PnP问题
 * 误差 = 观测像素 - 投影像素
 *      = z - project(T * point)
 */
void EdgeProjectXYZ2UVPoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    // 先将3D点变换到相机坐标系，再投影到像素坐标
    _error = _measurement - camera_->camera2pixel ( 
        pose->estimate().map(point_) );
}

/**
 * @brief EdgeProjectXYZ2UVPoseOnly的雅可比矩阵计算
 * 
 * 投影误差对位姿的雅可比矩阵
 * 
 * 链式法则: de/d(xi) = de/d(p_cam) * d(p_cam)/d(xi)
 * 
 * 其中:
 * - de/d(p_cam) 是投影函数对相机坐标的雅可比
 * - d(p_cam)/d(xi) 是相机坐标对位姿李代数的雅可比
 * 
 * 投影公式: u = fx * X/Z + cx, v = fy * Y/Z + cy
 * 需要求 d(u,v)/d(xi)
 */
void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    
    // 将3D点变换到相机坐标系
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;  // z的平方

    // 计算投影误差对位姿的雅可比矩阵
    // 这是标准的3D-2D投影雅可比推导结果
    // 对应g2o中SE3Expmap的李代数顺序
    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx_;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 *camera_->fx_;

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy_;
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy_;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy_;
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy_;
}


}
