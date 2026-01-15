/**
 * @file frame.h
 * @brief 帧类 - 表示视觉里程计中的一帧图像
 * 
 * 帧是视觉里程计的基本处理单元，包含：
 * - RGB彩色图像和深度图像
 * - 相机位姿（世界坐标系到相机坐标系的变换）
 * - 关键帧标识
 * - 时间戳等信息
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam 
{
    
// 前向声明
class MapPoint;

/**
 * @class Frame
 * @brief 帧类，表示RGBD相机采集的一帧数据
 * 
 * 存储图像数据、相机参数、位姿信息等，是视觉里程计处理的基本单元
 */
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;  // 智能指针类型定义
    
    // ==================== 帧属性 ====================
    unsigned long      id_;           // 帧的唯一标识符
    double             time_stamp_;   // 时间戳，记录该帧的采集时间
    Sophus::SE3d       T_c_w_;        // 相机位姿：从世界坐标系到相机坐标系的变换矩阵
                                      // T_c_w * P_w = P_c
    Camera::Ptr        camera_;       // 相机模型指针
    Mat                color_, depth_; // RGB彩色图像和深度图像
    bool               is_key_frame_; // 是否为关键帧
    
public:
    /**
     * @brief 默认构造函数
     */
    Frame();
    
    /**
     * @brief 完整构造函数
     * @param id 帧ID
     * @param time_stamp 时间戳
     * @param T_c_w 相机位姿
     * @param camera 相机模型
     * @param color 彩色图像
     * @param depth 深度图像
     */
    Frame( long id, double time_stamp=0, Sophus::SE3d T_c_w=Sophus::SE3d(), 
           Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    
    /**
     * @brief 析构函数
     */
    ~Frame();
    
    /**
     * @brief 工厂方法，创建新帧
     * @return 新帧的智能指针，ID自动递增
     */
    static Frame::Ptr createFrame(); 
    
    /**
     * @brief 获取特征点的深度值
     * @param kp 关键点
     * @return 深度值（米），若无效返回-1
     * 
     * 如果关键点位置深度为0，会检查周围4个邻域像素
     */
    double findDepth( const cv::KeyPoint& kp );
    
    /**
     * @brief 获取相机中心在世界坐标系中的位置
     * @return 相机光心的世界坐标
     * 
     * 相机中心 = T_c_w^(-1) 的平移部分
     */
    Vector3d getCamCenter() const;
    
    /**
     * @brief 设置帧的位姿
     * @param T_c_w 新的相机位姿
     */
    void setPose( const Sophus::SE3d& T_c_w );
    
    /**
     * @brief 判断一个世界坐标点是否在当前帧的视野内
     * @param pt_world 世界坐标系下的3D点
     * @return 是否在视野内
     * 
     * 检查条件：
     * 1. 点在相机前方（Z > 0）
     * 2. 投影到图像内（0 < u < width, 0 < v < height）
     */
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H
