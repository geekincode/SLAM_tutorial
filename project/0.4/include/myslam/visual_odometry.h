/**
 * @file visual_odometry.h
 * @brief 视觉里程计类 - SLAM系统的核心模块
 * 
 * 视觉里程计(Visual Odometry, VO)是SLAM系统的前端，负责：
 * 1. 从图像中提取特征点
 * 2. 与地图点进行特征匹配
 * 3. 估计相机位姿（PnP问题）
 * 4. 管理关键帧和地图点
 * 
 * 工作流程：
 * INITIALIZING -> OK -> (LOST)
 *                 |
 *              继续跟踪
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{

/**
 * @class VisualOdometry
 * @brief 视觉里程计类，负责追踪相机运动
 * 
 * 采用基于特征点的方法：
 * 1. ORB特征提取
 * 2. FLANN快速匹配
 * 3. PnP位姿估计 + g2o图优化
 */
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    
    /**
     * @brief 视觉里程计的状态机
     */
    enum VOState {
        INITIALIZING = -1,  // 初始化状态：等待第一帧
        OK = 0,             // 正常跟踪状态
        LOST                // 跟踪丢失状态
    };
    
    // ==================== 状态变量 ====================
    VOState     state_;     // 当前VO状态
    Map::Ptr    map_;       // 地图指针，包含所有地图点和关键帧
    
    Frame::Ptr  ref_;       // 参考关键帧
    Frame::Ptr  curr_;      // 当前处理帧
    
    // ==================== 特征相关 ====================
    cv::Ptr<cv::ORB> orb_;                  // ORB特征检测器和描述子计算器
    vector<cv::KeyPoint>    keypoints_curr_;  // 当前帧的关键点
    Mat                     descriptors_curr_; // 当前帧的描述子
    
    cv::FlannBasedMatcher   matcher_flann_;    // FLANN快速近似最近邻匹配器
    vector<MapPoint::Ptr>   match_3dpts_;      // 匹配成功的3D地图点
    vector<int>             match_2dkp_index_; // 匹配成功的2D关键点索引（在keypoints_curr_中的索引）
   
    // ==================== 位姿估计结果 ====================
    Sophus::SE3d T_c_w_estimated_;  // 估计的当前帧位姿（世界到相机的变换）
    int num_inliers_;               // PnP求解中的内点数量
    int num_lost_;                  // 连续跟踪丢失的帧数
    
    // ==================== 算法参数 ====================
    // 这些参数从配置文件中读取
    int num_of_features_;       // ORB特征数量
    double scale_factor_;       // 图像金字塔的缩放因子
    int level_pyramid_;         // 金字塔层数
    float match_ratio_;         // 特征匹配的距离比率阈值
    int max_num_lost_;          // 最大连续丢失帧数，超过则判定为LOST
    int min_inliers_;           // PnP最小内点数，少于则拒绝该位姿
    double key_frame_min_rot;   // 关键帧选取的最小旋转角度
    double key_frame_min_trans; // 关键帧选取的最小平移距离
    double map_point_erase_ratio_;  // 地图点删除的匹配率阈值
    
public:
    /**
     * @brief 构造函数，从配置文件初始化参数
     */
    VisualOdometry();
    
    /**
     * @brief 析构函数
     */
    ~VisualOdometry();
    
    /**
     * @brief 添加新帧到VO系统
     * @param frame 新的帧
     * @return 是否成功处理
     * 
     * 这是VO的主入口函数，根据当前状态执行不同操作：
     * - INITIALIZING: 初始化地图
     * - OK: 正常追踪
     * - LOST: 输出丢失信息
     */
    bool addFrame( Frame::Ptr frame );
    
protected:
    // ==================== 内部处理函数 ====================
    
    /**
     * @brief 提取当前帧的ORB关键点
     */
    void extractKeyPoints();
    
    /**
     * @brief 计算关键点的ORB描述子
     */
    void computeDescriptors(); 
    
    /**
     * @brief 特征匹配
     * 
     * 将当前帧的描述子与地图中可见的地图点描述子进行匹配
     * 使用FLANN进行快速近似最近邻搜索
     */
    void featureMatching();
    
    /**
     * @brief PnP位姿估计
     * 
     * 使用RANSAC + PnP求解初始位姿，然后用g2o进行Bundle Adjustment优化
     */
    void poseEstimationPnP(); 
    
    /**
     * @brief 优化地图
     * 
     * 删除质量差的地图点：
     * - 不在视野内的点
     * - 匹配率过低的点
     * - 观测角度过大的点
     */
    void optimizeMap();
    
    /**
     * @brief 添加关键帧
     * 
     * 将当前帧设为关键帧，并添加新的地图点
     */
    void addKeyFrame();
    
    /**
     * @brief 添加新的地图点
     * 
     * 将当前帧中未匹配的特征点三角化为新的地图点
     */
    void addMapPoints();
    
    /**
     * @brief 检查估计的位姿是否有效
     * @return 是否有效
     * 
     * 检查条件：
     * - 内点数量是否足够
     * - 运动是否过大（可能是错误匹配）
     */
    bool checkEstimatedPose(); 
    
    /**
     * @brief 检查是否需要插入新的关键帧
     * @return 是否需要新关键帧
     * 
     * 判断条件：相对于参考帧的旋转或平移超过阈值
     */
    bool checkKeyFrame();
    
    /**
     * @brief 计算帧与地图点之间的观测角度
     * @param frame 帧
     * @param point 地图点
     * @return 观测角度（弧度）
     * 
     * 用于判断该地图点是否仍然适合在当前视角下进行匹配
     */
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    
};
}

#endif // VISUALODOMETRY_H
