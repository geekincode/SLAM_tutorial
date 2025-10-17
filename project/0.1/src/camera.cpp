/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/camera.h"
#include <sophus/se3.hpp>

namespace myslam
{

Camera::Camera()
{
}


Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3d& T_c_w )
{
    // T_c_w 是从世界坐标系到相机坐标系的变换矩阵（SE3 类型）
    return T_c_w*p_w;
}

Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3d& T_c_w )
{
    // 使用 T_c_w 的逆矩阵实现反向变换
    return T_c_w.inverse() *p_c;
}

/**
 * @brief 相机坐标转像素坐标
 * 
 * @param p_c 相机坐标
 * @return 像素坐标
 * @note
    作用：将相机坐标 (X, Y, Z) 投影到像素坐标 (u, v)。
    数学公式（针孔模型）：
    u = fx_ * X / Z + cx_
    v = fy_ * Y / Z + cy_
 */

Vector2d Camera::camera2pixel ( const Vector3d& p_c )
{
    // p_c(0,0) 是 Eigen 矩阵元素的标准访问方式，明确指定行索引和列索引（格式为 (row, column)）。所以p_c(0,0)是x坐标
    return Vector2d (
        fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
        fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
    );
}

/**
 * @brief 像素坐标转相机坐标
 *
 * @param p_p 像素坐标 (u, v)
 * @param depth 深度值
 * @return 相机坐标
 * @note
    作用：将像素坐标 (u, v) 反投影到相机坐标，需已知深度 d（即 Z 值）。
    数学公式：
    X = (u - cx_) * d / fx_
    Y = (v - cy_) * d / fy_
    Z = d
 */
Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
{
    return Vector3d (
        ( p_p ( 0,0 )-cx_ ) *depth/fx_,
        ( p_p ( 1,0 )-cy_ ) *depth/fy_,
        depth
    );
}


/**
 * @brief 世界坐标转像素坐标
 * @param p_w 世界坐标
 * @param T_c_w 相机到世界坐标的变换矩阵
 * @return 像素坐标
 * @note
    先调用 world2camera 转到相机坐标，再调用 camera2pixel 投影到像素。
 */
Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3d& T_c_w )
{
    return camera2pixel ( world2camera ( p_w, T_c_w ) );
}


/**
 * @brief 像素坐标转世界坐标
 * @param p_p 像素坐标
 * @param T_c_w 相机到世界坐标的变换矩阵
 * @param depth 深度值
 * @return 世界坐标
 * @note
    先调用 pixel2camera（需深度 d）得到相机坐标，再调用 camera2world 转到世界坐标。
 */
Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3d& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


}
