/**
 * @file map.cpp
 * @brief 地图类的实现
 * 
 * 实现关键帧和地图点的管理功能
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#include "myslam/map.h"

namespace myslam
{

/**
 * @brief 插入关键帧
 * 
 * 如果该ID的关键帧已存在，则更新；否则新增。
 * 使用unordered_map实现O(1)的平均插入和查询复杂度。
 * 
 * @param frame 要插入的关键帧
 */
void Map::insertKeyFrame ( Frame::Ptr frame )
{
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    
    // 检查关键帧是否已存在
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        // 不存在，插入新关键帧
        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        // 已存在，更新关键帧
        keyframes_[ frame->id_ ] = frame;
    }
}

/**
 * @brief 插入地图点
 * 
 * 如果该ID的地图点已存在，则更新；否则新增。
 * 
 * @param map_point 要插入的地图点
 */
void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    // 检查地图点是否已存在
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        // 不存在，插入新地图点
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        // 已存在，更新地图点
        map_points_[map_point->id_] = map_point;
    }
}


}
