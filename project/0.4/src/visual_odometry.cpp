/**
 * @file visual_odometry.cpp
 * @brief 视觉里程计类的实现
 * 
 * 实现了完整的视觉里程计流程：
 * 1. 特征提取（ORB）
 * 2. 特征匹配（FLANN）
 * 3. 位姿估计（PnP + BA优化）
 * 4. 关键帧选取和地图管理
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"

namespace myslam
{

/**
 * @brief 构造函数
 * 
 * 从配置文件初始化所有参数，创建ORB特征检测器和FLANN匹配器
 */
VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{
    // 从配置文件读取参数
    num_of_features_    = Config::get<int> ( "number_of_features" );      // ORB特征点数量
    scale_factor_       = Config::get<double> ( "scale_factor" );         // 图像金字塔缩放因子
    level_pyramid_      = Config::get<int> ( "level_pyramid" );           // 金字塔层数
    match_ratio_        = Config::get<float> ( "match_ratio" );           // 匹配距离比率阈值
    max_num_lost_       = Config::get<float> ( "max_num_lost" );          // 最大连续丢失帧数
    min_inliers_        = Config::get<int> ( "min_inliers" );             // 最小内点数
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );    // 关键帧最小旋转
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" ); // 关键帧最小平移
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" ); // 地图点删除阈值
    
    // 创建ORB特征检测器
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

/**
 * @brief 析构函数
 */
VisualOdometry::~VisualOdometry()
{

}

/**
 * @brief 添加新帧到VO系统
 * 
 * 这是VO的主入口函数，根据当前状态执行不同操作：
 * - INITIALIZING: 初始化，将第一帧作为关键帧，提取特征建立初始地图
 * - OK: 正常跟踪，特征匹配->PnP位姿估计->地图更新
 * - LOST: 跟踪丢失，输出信息
 * 
 * @param frame 新输入的帧
 * @return 是否成功处理该帧
 */
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:
    {
        // ========== 初始化阶段 ==========
        state_ = OK;
        curr_ = ref_ = frame;  // 将第一帧设为参考帧和当前帧
        
        // 从第一帧提取特征并建立初始地图
        extractKeyPoints();
        computeDescriptors();
        addKeyFrame();      // 第一帧是关键帧
        break;
    }
    case OK:
    {
        // ========== 正常跟踪阶段 ==========
        curr_ = frame;
        // 用参考帧的位姿作为当前帧位姿的初始估计
        curr_->T_c_w_ = ref_->T_c_w_;
        
        // 特征提取和匹配
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        
        // PnP位姿估计
        poseEstimationPnP();
        
        if ( checkEstimatedPose() == true ) // 位姿估计有效
        {
            curr_->T_c_w_ = T_c_w_estimated_;  // 更新当前帧位姿
            optimizeMap();    // 优化地图（删除低质量点）
            num_lost_ = 0;    // 重置丢失计数
            
            if ( checkKeyFrame() == true ) // 需要新关键帧
            {
                addKeyFrame();
            }
        }
        else // 位姿估计失败
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;  // 连续丢失过多，判定为跟踪丢失
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        // ========== 丢失状态 ==========
        cout<<"vo has lost."<<endl;
        break;
    }
    }

    return true;
}

/**
 * @brief 提取当前帧的ORB关键点
 * 
 * 使用ORB算法检测图像中的特征点
 * ORB特征具有旋转不变性和一定的尺度不变性
 */
void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect ( curr_->color_, keypoints_curr_ );
    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}

/**
 * @brief 计算关键点的ORB描述子
 * 
 * ORB描述子是256位的二进制描述子，可以使用汉明距离快速匹配
 */
void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
}

/**
 * @brief 特征匹配
 * 
 * 将当前帧的特征与地图中可见的地图点进行匹配：
 * 1. 筛选在当前帧视野内的地图点作为候选
 * 2. 使用FLANN进行快速近似最近邻匹配
 * 3. 根据距离比率筛选优质匹配
 */
void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    
    // ========== 1. 筛选候选地图点 ==========
    Mat desp_map;  // 候选地图点的描述子矩阵
    vector<MapPoint::Ptr> candidate;  // 候选地图点
    
    for ( auto& allpoints: map_->map_points_ )
    {
        MapPoint::Ptr& p = allpoints.second;
        // 检查地图点是否在当前帧视野内
        if ( curr_->isInFrame(p->pos_) )
        {
            p->visible_times_++;  // 增加可见计数
            candidate.push_back( p );
            desp_map.push_back( p->descriptor_ );
        }
    }
    
    // ========== 2. FLANN匹配 ==========
    // FLANN使用LSH(局部敏感哈希)加速二进制描述子的匹配
    matcher_flann_.match ( desp_map, descriptors_curr_, matches );
    
    // ========== 3. 筛选优质匹配 ==========
    // 找到最小匹配距离
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    // 根据距离阈值筛选
    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches )
    {
        // 距离小于 min_dis * match_ratio 或 30 的匹配被保留
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            match_3dpts_.push_back( candidate[m.queryIdx] );  // 匹配的3D点
            match_2dkp_index_.push_back( m.trainIdx );         // 对应的2D关键点索引
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed() <<endl;
}

/**
 * @brief PnP位姿估计
 * 
 * 使用3D-2D匹配对进行位姿估计：
 * 1. 构建3D-2D对应关系
 * 2. 使用RANSAC + PnP求解初始位姿
 * 3. 使用g2o进行Bundle Adjustment优化
 * 
 * PnP问题：已知n个3D点及其在图像上的2D投影，求解相机位姿
 */
void VisualOdometry::poseEstimationPnP()
{
    // ========== 1. 构建3D-2D对应关系 ==========
    vector<cv::Point3f> pts3d;  // 3D点（世界坐标系）
    vector<cv::Point2f> pts2d;  // 2D点（像素坐标）

    // 收集匹配的2D点
    for ( int index:match_2dkp_index_ )
    {
        pts2d.push_back ( keypoints_curr_[index].pt );
    }
    // 收集匹配的3D点
    for ( MapPoint::Ptr pt:match_3dpts_ )
    {
        pts3d.push_back( pt->getPositionCV() );
    }

    // ========== 2. 构建相机内参矩阵 ==========
    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0,0,1
            );
    
    // ========== 3. RANSAC + PnP求解 ==========
    Mat rvec, tvec, inliers;
    // solvePnPRansac: 使用RANSAC剔除外点，提高鲁棒性
    // 参数: 3D点, 2D点, 内参K, 畸变系数(空), 旋转向量, 平移向量
    // false: 不使用初始估计, 100: 迭代次数, 4.0: 重投影误差阈值
    // 0.99: 置信度, inliers: 输出内点索引
    cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    
    // ========== 4. 构建SE3位姿 ==========
    // 将旋转向量转换为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    // 转换为Eigen矩阵
    Eigen::Matrix3d r_mat;
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            r_mat(i, j) = R.at<double>(i, j);
    
    // 构建SE3变换（从世界到相机）
    T_c_w_estimated_ = Sophus::SE3d (
                           Sophus::SO3d ( r_mat ),
                           Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                       );

    // ========== 5. g2o图优化 ==========
    // 使用Bundle Adjustment进一步优化位姿
    
    // 创建块求解器: 6维位姿, 2维重投影误差
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block (std::unique_ptr<Block::LinearSolverType>(linearSolver));
    
    // 使用Levenberg-Marquardt算法
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (std::unique_ptr<Block>(solver_ptr));
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // 添加位姿顶点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotationMatrix(), T_c_w_estimated_.translation()
    ));
    optimizer.addVertex ( pose );

    // 添加重投影边（仅使用内点）
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int> ( i,0 );
        
        // 创建3D->2D投影边
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId ( i );
        edge->setVertex ( 0, pose );  // 连接到位姿顶点
        edge->camera_ = curr_->camera_.get();  // 相机模型
        edge->point_ = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );  // 3D点
        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );  // 观测的2D像素
        edge->setInformation ( Eigen::Matrix2d::Identity() );  // 信息矩阵（权重）
        optimizer.addEdge ( edge );
        
        // 更新内点地图点的匹配计数
        match_3dpts_[index]->matched_times_++;
    }

    // 执行优化
    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );  // 迭代10次

    // 取出优化后的位姿
    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
    
    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
}

/**
 * @brief 检查估计的位姿是否有效
 * 
 * 验证条件：
 * 1. 内点数量是否足够
 * 2. 相对运动是否过大（过大可能是误匹配）
 * 
 * @return 位姿是否有效
 */
bool VisualOdometry::checkEstimatedPose()
{
    // 条件1: 检查内点数量
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    
    // 条件2: 检查相对运动是否过大
    // 计算相对于参考帧的相对运动
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();  // 转换为李代数（6维向量）
    
    if ( d.norm() > 5.0 )  // 李代数范数过大
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}

/**
 * @brief 检查是否需要插入新的关键帧
 * 
 * 判断条件：相对于参考帧的旋转或平移超过阈值
 * 这确保关键帧之间有足够的视差，有利于三角化和定位
 * 
 * @return 是否需要新关键帧
 */
bool VisualOdometry::checkKeyFrame()
{
    // 计算相对运动
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    
    // 分离平移和旋转部分
    Vector3d trans = d.head<3>();  // 前3维是平移
    Vector3d rot = d.tail<3>();    // 后3维是旋转（角轴表示）
    
    // 旋转或平移超过阈值则认为需要新关键帧
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

/**
 * @brief 添加关键帧
 * 
 * 将当前帧设为关键帧，更新参考帧
 * 如果是第一个关键帧，还需要将所有特征点三角化为地图点
 */
void VisualOdometry::addKeyFrame()
{
    if ( map_->keyframes_.empty() )
    {
        // ========== 第一个关键帧：建立初始地图 ==========
        // 将所有有效深度的特征点添加为地图点
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            // 获取特征点的深度
            double d = curr_->findDepth ( keypoints_curr_[i] );
            if ( d < 0 ) 
                continue;  // 深度无效，跳过
            
            // 将像素坐标反投影为世界坐标
            Vector3d p_world = ref_->camera_->pixel2world (
                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
            );
            
            // 计算观测方向（从相机中心指向地图点）
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            
            // 创建地图点
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );
            
            // 插入地图
            map_->insertMapPoint( map_point );
        }
    }
    
    // 将当前帧插入关键帧列表
    map_->insertKeyFrame ( curr_ );
    // 更新参考帧为当前帧
    ref_ = curr_;
}

void VisualOdometry::addMapPoints()
{
    // add the new map points into map
    vector<bool> matched(keypoints_curr_.size(), false); 
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        if ( matched[i] == true )   
            continue;
        double d = curr_->findDepth ( keypoints_curr_[i] );
        if ( d<0 )  
            continue;
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
            curr_->T_c_w_, d
        );
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );
        map_->insertMapPoint( map_point );
    }
}

void VisualOdometry::optimizeMap()
{
    // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
    
    if ( match_2dkp_index_.size()<100 )
        addMapPoints();
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}

double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}


}
