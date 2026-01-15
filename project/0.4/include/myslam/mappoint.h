/**
 * @file mappoint.h
 * @brief 地图点类 - 表示三维空间中的一个路标点
 * 
 * 地图点是SLAM系统中的基本地图元素，代表环境中的一个3D特征点。
 * 每个地图点包含：
 * - 3D位置坐标
 * - 特征描述子（用于匹配）
 * - 观测信息（哪些帧可以看到该点）
 * - 质量评估信息
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
    
class Frame;  // 前向声明

/**
 * @class MapPoint
 * @brief 地图点类，表示3D空间中的一个特征路标点
 * 
 * 地图点是视觉SLAM建立地图的基本单元，用于定位和建图
 */
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;       // 智能指针类型定义
    
    // ==================== 地图点属性 ====================
    unsigned long      id_;                  // 地图点的唯一标识符
    static unsigned long factory_id_;        // 工厂ID，用于自动生成唯一ID
    bool               good_;                // 是否为有效点（质量好的点）
    Vector3d           pos_;                 // 世界坐标系下的3D位置
    Vector3d           norm_;                // 平均观测方向的单位向量
                                             // 用于判断观测角度是否过大
    Mat                descriptor_;          // 特征描述子，用于特征匹配
    
    // ==================== 观测信息 ====================
    list<Frame*>       observed_frames_;     // 能够观测到该点的关键帧列表
    int                matched_times_;       // 匹配次数：在位姿估计中作为内点的次数
    int                visible_times_;       // 可见次数：在当前帧视野内的次数
                                             // match_ratio = matched_times_ / visible_times_
                                             // 用于评估地图点的质量

    /**
     * @brief 默认构造函数
     */
    MapPoint();
    
    /**
     * @brief 完整构造函数
     * @param id 地图点ID
     * @param position 世界坐标系下的位置
     * @param norm 观测方向单位向量
     * @param frame 首次观测到该点的帧
     * @param descriptor 特征描述子
     */
    MapPoint( 
        unsigned long id, 
        const Vector3d& position, 
        const Vector3d& norm, 
        Frame* frame=nullptr, 
        const Mat& descriptor=Mat() 
    );
    
    /**
     * @brief 获取OpenCV格式的3D坐标
     * @return cv::Point3f格式的坐标
     */
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }
    
    /**
     * @brief 工厂方法，创建空的地图点
     * @return 新地图点的智能指针
     */
    static MapPoint::Ptr createMapPoint();
    
    /**
     * @brief 工厂方法，创建带完整信息的地图点
     * @param pos_world 世界坐标系下的位置
     * @param norm_ 观测方向单位向量
     * @param descriptor 特征描述子
     * @param frame 观测到该点的帧
     * @return 新地图点的智能指针
     */
    static MapPoint::Ptr createMapPoint( 
        const Vector3d& pos_world, 
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame );
};
}

#endif // MAPPOINT_H
