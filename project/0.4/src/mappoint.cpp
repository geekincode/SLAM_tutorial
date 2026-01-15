/**
 * @file mappoint.cpp
 * @brief 地图点类的实现
 * 
 * 实现地图点的创建和工厂方法
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#include "myslam/common_include.h"
#include "myslam/mappoint.h"

namespace myslam
{

/**
 * @brief 默认构造函数
 * 
 * 创建一个空的地图点，位置和方向都初始化为零向量
 */
MapPoint::MapPoint()
: id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
{

}

/**
 * @brief 完整构造函数
 * 
 * @param id 地图点ID
 * @param position 世界坐标系下的3D位置
 * @param norm 观测方向的单位向量（从相机中心指向地图点）
 * @param frame 首次观测到该点的帧
 * @param descriptor 特征描述子
 */
MapPoint::MapPoint ( long unsigned int id, const Vector3d& position, const Vector3d& norm, Frame* frame, const Mat& descriptor )
: id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
{
    // 将观测到该点的帧加入列表
    observed_frames_.push_back(frame);
}

/**
 * @brief 工厂方法：创建空地图点
 * 
 * 使用factory_id_自动生成唯一ID
 * 
 * @return 新地图点的智能指针
 */
MapPoint::Ptr MapPoint::createMapPoint()
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) )
    );
}

/**
 * @brief 工厂方法：创建完整的地图点
 * 
 * 包含位置、方向、描述子和观测帧信息
 * 
 * @param pos_world 世界坐标系下的位置
 * @param norm 观测方向单位向量
 * @param descriptor 特征描述子
 * @param frame 观测到该点的帧
 * @return 新地图点的智能指针
 */
MapPoint::Ptr MapPoint::createMapPoint ( 
    const Vector3d& pos_world, 
    const Vector3d& norm, 
    const Mat& descriptor, 
    Frame* frame )
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )
    );
}

// 静态成员初始化：工厂ID从0开始递增
unsigned long MapPoint::factory_id_ = 0;

}
