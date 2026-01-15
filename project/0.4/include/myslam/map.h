/**
 * @file map.h
 * @brief 地图类 - 管理所有地图点和关键帧
 * 
 * 地图是SLAM系统的核心数据结构，存储和管理：
 * - 所有的地图点（3D路标点）
 * - 所有的关键帧
 * 
 * 使用哈希表（unordered_map）存储，支持快速的插入和查询
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{

/**
 * @class Map
 * @brief 地图类，管理SLAM系统中的所有地图点和关键帧
 * 
 * 提供地图点和关键帧的插入、删除、查询功能
 */
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    
    // ==================== 地图数据 ====================
    // 使用unordered_map存储，key为ID，value为智能指针
    // 优点：O(1)的平均查询和插入复杂度
    unordered_map<unsigned long, MapPoint::Ptr>  map_points_;  // 所有地图点（路标点）
    unordered_map<unsigned long, Frame::Ptr>     keyframes_;   // 所有关键帧

    /**
     * @brief 默认构造函数
     */
    Map() {}
    
    /**
     * @brief 插入关键帧
     * @param frame 要插入的关键帧
     * 
     * 如果该ID的关键帧已存在，则更新；否则新增
     */
    void insertKeyFrame( Frame::Ptr frame );
    
    /**
     * @brief 插入地图点
     * @param map_point 要插入的地图点
     * 
     * 如果该ID的地图点已存在，则更新；否则新增
     */
    void insertMapPoint( MapPoint::Ptr map_point );
};
}

#endif // MAP_H
