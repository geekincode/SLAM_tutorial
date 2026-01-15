/**
 * @file camera.h
 * @brief 相机模型类 - 针孔RGBD相机模型
 * 
 * 本文件定义了针孔相机模型类，实现了不同坐标系之间的转换：
 * - 世界坐标系 (World): 全局参考坐标系
 * - 相机坐标系 (Camera): 以相机光心为原点的坐标系
 * - 像素坐标系 (Pixel): 图像平面上的2D坐标
 * 
 * 针孔相机模型的投影关系：
 *   u = fx * X/Z + cx
 *   v = fy * Y/Z + cy
 * 
 * 其中 (fx, fy) 是焦距，(cx, cy) 是主点坐标
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{

/**
 * @class Camera
 * @brief 针孔RGBD相机模型类
 * 
 * 封装了相机的内参和坐标转换功能，支持RGB-D相机的深度缩放
 */
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;  // 智能指针类型定义
    
    // ==================== 相机内参 ====================
    float   fx_, fy_;       // 焦距 (focal length)，单位：像素
    float   cx_, cy_;       // 主点坐标 (principal point)，通常接近图像中心
    float   depth_scale_;   // 深度缩放因子，用于将深度图像素值转换为实际深度（米）

    /**
     * @brief 默认构造函数，从配置文件读取相机参数
     */
    Camera();
    
    /**
     * @brief 参数化构造函数
     * @param fx 水平方向焦距
     * @param fy 垂直方向焦距
     * @param cx 主点x坐标
     * @param cy 主点y坐标
     * @param depth_scale 深度缩放因子（默认为0）
     */
    Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
    {}

    // ==================== 坐标转换函数 ====================
    
    /**
     * @brief 世界坐标系 -> 相机坐标系
     * @param p_w 世界坐标系下的3D点
     * @param T_c_w 相机位姿（世界到相机的变换）
     * @return 相机坐标系下的3D点
     * 
     * 公式: P_c = T_c_w * P_w
     */
    Vector3d world2camera( const Vector3d& p_w, const Sophus::SE3d& T_c_w );
    
    /**
     * @brief 相机坐标系 -> 世界坐标系
     * @param p_c 相机坐标系下的3D点
     * @param T_c_w 相机位姿（世界到相机的变换）
     * @return 世界坐标系下的3D点
     * 
     * 公式: P_w = T_c_w^(-1) * P_c
     */
    Vector3d camera2world( const Vector3d& p_c, const Sophus::SE3d& T_c_w );
    
    /**
     * @brief 相机坐标系 -> 像素坐标系
     * @param p_c 相机坐标系下的3D点
     * @return 像素坐标 (u, v)
     * 
     * 公式: u = fx * X/Z + cx, v = fy * Y/Z + cy
     */
    Vector2d camera2pixel( const Vector3d& p_c );
    
    /**
     * @brief 像素坐标系 -> 相机坐标系
     * @param p_p 像素坐标 (u, v)
     * @param depth 该点的深度值（默认为1）
     * @return 相机坐标系下的3D点
     * 
     * 公式: X = (u-cx)*depth/fx, Y = (v-cy)*depth/fy, Z = depth
     */
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
    
    /**
     * @brief 像素坐标系 -> 世界坐标系
     * @param p_p 像素坐标 (u, v)
     * @param T_c_w 相机位姿
     * @param depth 该点的深度值
     * @return 世界坐标系下的3D点
     */
    Vector3d pixel2world ( const Vector2d& p_p, const Sophus::SE3d& T_c_w, double depth=1 );
    
    /**
     * @brief 世界坐标系 -> 像素坐标系
     * @param p_w 世界坐标系下的3D点
     * @param T_c_w 相机位姿
     * @return 像素坐标 (u, v)
     */
    Vector2d world2pixel ( const Vector3d& p_w, const Sophus::SE3d& T_c_w );

};

}
#endif // CAMERA_H
