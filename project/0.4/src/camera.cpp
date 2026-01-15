/**
 * @file camera.cpp
 * @brief 相机模型类的实现
 * 
 * 实现针孔相机模型的各种坐标转换函数：
 * - 世界坐标系 <-> 相机坐标系 <-> 像素坐标系
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#include "myslam/camera.h"
#include <myslam/config.h>

namespace myslam
{
    using Sophus::SE3d;
    
/**
 * @brief 默认构造函数
 * 
 * 从配置文件中读取相机内参：
 * - fx, fy: 焦距
 * - cx, cy: 主点坐标
 * - depth_scale: 深度缩放因子
 */
Camera::Camera()
{
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}

/**
 * @brief 世界坐标系 -> 相机坐标系
 * 
 * 变换公式: P_c = T_c_w * P_w
 * 
 * @param p_w 世界坐标系下的3D点
 * @param T_c_w 从世界坐标系到相机坐标系的变换矩阵 (SE3)
 * @return 相机坐标系下的3D点
 */
Vector3d Camera::world2camera ( const Vector3d& p_w, const Sophus::SE3d& T_c_w )
{
    return T_c_w*p_w;
}

/**
 * @brief 相机坐标系 -> 世界坐标系
 * 
 * 变换公式: P_w = T_c_w^(-1) * P_c = T_w_c * P_c
 * 
 * @param p_c 相机坐标系下的3D点
 * @param T_c_w 从世界坐标系到相机坐标系的变换矩阵
 * @return 世界坐标系下的3D点
 */
Vector3d Camera::camera2world ( const Vector3d& p_c, const Sophus::SE3d& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

/**
 * @brief 相机坐标系 -> 像素坐标系
 * 
 * 针孔相机投影模型：
 *   u = fx * X/Z + cx
 *   v = fy * Y/Z + cy
 * 
 * 其中 (X, Y, Z) 是相机坐标系下的3D点坐标
 * 
 * @param p_c 相机坐标系下的3D点 (X, Y, Z)
 * @return 像素坐标 (u, v)
 */
Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    return Vector2d (
               fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,  // u = fx * X/Z + cx
               fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_   // v = fy * Y/Z + cy
           );
}

/**
 * @brief 像素坐标系 -> 相机坐标系
 * 
 * 反投影模型：
 *   X = (u - cx) * depth / fx
 *   Y = (v - cy) * depth / fy
 *   Z = depth
 * 
 * @param p_p 像素坐标 (u, v)
 * @param depth 该点的深度值
 * @return 相机坐标系下的3D点
 */
Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
    return Vector3d (
               ( p_p ( 0,0 )-cx_ ) *depth/fx_,  // X = (u - cx) * depth / fx
               ( p_p ( 1,0 )-cy_ ) *depth/fy_,  // Y = (v - cy) * depth / fy
               depth                             // Z = depth
           );
}

/**
 * @brief 世界坐标系 -> 像素坐标系
 * 
 * 组合变换: 先转到相机坐标系，再投影到像素坐标系
 * 
 * @param p_w 世界坐标系下的3D点
 * @param T_c_w 相机位姿
 * @return 像素坐标
 */
Vector2d Camera::world2pixel ( const Vector3d& p_w, const Sophus::SE3d& T_c_w )
{
    return camera2pixel ( world2camera(p_w, T_c_w) );
}

/**
 * @brief 像素坐标系 -> 世界坐标系
 * 
 * 组合变换: 先反投影到相机坐标系，再转到世界坐标系
 * 
 * @param p_p 像素坐标
 * @param T_c_w 相机位姿
 * @param depth 深度值
 * @return 世界坐标系下的3D点
 */
Vector3d Camera::pixel2world ( const Vector2d& p_p, const Sophus::SE3d& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


}
