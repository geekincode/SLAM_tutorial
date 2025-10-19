/*
 * 视觉里程计实现文件
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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

namespace myslam
{

// VisualOdometry构造函数，初始化各项参数和ORB特征检测器
VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ),     // 初始状态设为初始化状态
    ref_ ( nullptr ),            // 参考帧指针初始化为空
    curr_ ( nullptr ),           // 当前帧指针初始化为空
    map_ ( new Map ),            // 创建地图对象
    num_lost_ ( 0 ),             // 丢失帧计数器初始化为0
    num_inliers_ ( 0 )           // 内点计数器初始化为0
{
    // 从配置文件中读取各项参数
    num_of_features_    = Config::get<int> ( "number_of_features" );     // 特征点数量
    scale_factor_       = Config::get<double> ( "scale_factor" );        // 图像金字塔缩放因子
    level_pyramid_      = Config::get<int> ( "level_pyramid" );          // 图像金字塔层数
    match_ratio_        = Config::get<float> ( "match_ratio" );          // 特征匹配比例阈值
    max_num_lost_       = Config::get<float> ( "max_num_lost" );         // 最大连续丢失帧数
    min_inliers_        = Config::get<int> ( "min_inliers" );            // 最小内点数
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );   // 关键帧最小旋转角度
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );// 关键帧最小平移距离
    
    // 创建ORB特征检测器实例
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

// VisualOdometry析构函数
VisualOdometry::~VisualOdometry()
{

}

/**
 * @brief 添加新的帧并进行视觉里程计处理
 * @param frame 新的图像帧
 * @return 处理是否成功
 */
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:  // 初始化状态
    {
        state_ = OK;                    // 设置状态为正常跟踪状态
        curr_ = ref_ = frame;           // 将第一帧同时设为当前帧和参考帧
        map_->insertKeyFrame ( frame ); // 将第一帧插入地图作为第一个关键帧
        
        // 提取第一帧的特征点
        // extract features from first frame 
        extractKeyPoints();
        // 计算特征点的描述子
        computeDescriptors();
        // 计算参考帧中特征点的3D坐标
        // compute the 3d position of features in ref frame 
        setRef3DPoints();
        break;
    }
    case OK:  // 正常跟踪状态
    {
        curr_ = frame;                 // 更新当前帧
        
        // 提取当前帧的特征点和描述子
        extractKeyPoints();
        computeDescriptors();
        
        // 进行特征匹配
        featureMatching();
        
        // 使用PnP算法估计位姿
        poseEstimationPnP();
        
        // 检查估计的位姿是否有效
        if ( checkEstimatedPose() == true ) // 位姿估计有效
        {
            // 更新当前帧的位姿：T_c_w = T_c_r * T_r_w
            // 即当前帧到世界的变换 = 当前帧到参考帧的变换 * 参考帧到世界的变换
            curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  
            
            ref_ = curr_;              // 将当前帧设为新的参考帧
            setRef3DPoints();          // 重新计算参考帧3D点
            num_lost_ = 0;             // 重置丢失计数器
            
            // 检查是否需要添加关键帧
            if ( checkKeyFrame() == true ) // 是关键帧
            {
                addKeyFrame();         // 添加关键帧到地图  is a key-frame
            }
        }
        else // 位姿估计无效  bad estimation due to various reasons
        {
            num_lost_++;               // 增加丢失计数
            if ( num_lost_ > max_num_lost_ ) // 连续丢失帧数超过阈值
            {
                state_ = LOST;         // 设置状态为丢失状态
            }
            return false;              // 返回处理失败
        }
        break;
    }
    case LOST:  // 跟踪丢失状态
    {
        cout<<"vo has lost."<<endl;   // 输出提示信息
        break;
    }
    }

    return true;  // 返回处理成功
}

/**
 * @brief 提取当前帧的特征点
 */
void VisualOdometry::extractKeyPoints()
{
    // 使用ORB检测器检测当前帧彩色图像中的关键点
    orb_->detect ( curr_->color_, keypoints_curr_ );
}

/**
 * @brief 计算特征点的描述子
 */
void VisualOdometry::computeDescriptors()
{
    // 使用ORB计算当前帧关键点的描述子
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
}

/**
 * @brief 进行特征匹配
 */
void VisualOdometry::featureMatching()
{
    // 使用OpenCV的暴力匹配器进行特征匹配
    // match desp_ref and desp_curr, use OpenCV's brute force match 
    vector<cv::DMatch> matches;                      // 存储所有匹配结果
    cv::BFMatcher matcher ( cv::NORM_HAMMING );      // 创建汉明距离匹配器
    // 匹配参考帧和当前帧的描述子
    matcher.match ( descriptors_ref_, descriptors_curr_, matches );
    
    // 找到最小匹配距离
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    feature_matches_.clear();  // 清空之前的匹配结果
    
    // 根据阈值筛选优质匹配点
    for ( cv::DMatch& m : matches )
    {
        // 距离小于阈值的匹配点被认为是好的匹配
        // 阈值为min_dis*match_ratio_和30.0中的较大值
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            feature_matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;  // 输出优质匹配点数量
}

/**
 * @brief 设置参考帧中的3D点
 */
void VisualOdometry::setRef3DPoints()
{
    // 清空之前的3D点和描述子
    // select the features with depth measurements 
    pts_3d_ref_.clear();
    descriptors_ref_ = Mat();
    
    // 遍历当前帧的所有关键点
    for ( size_t i=0; i<keypoints_curr_.size(); i++ )
    {
        // 获取该关键点对应的深度值
        double d = ref_->findDepth(keypoints_curr_[i]);               
        if ( d > 0)  // 如果深度值有效
        {
            // 将像素坐标和深度值转换为相机坐标系下的3D点
            Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );
            
            // 添加3D点到参考帧3D点列表
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            // 添加对应的描述子
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }
}

/**
 * @brief 使用PnP算法估计位姿
 */
void VisualOdometry::poseEstimationPnP()
{
    // 构造用于PnP的3D-2D点对
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;   // 3D点（参考帧坐标系）
    vector<cv::Point2f> pts2d;   // 2D点（当前帧像素坐标）
    
    // 遍历所有特征匹配
    for ( cv::DMatch m:feature_matches_ )
    {
        // 添加匹配点对应的3D点和2D点
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );       // 参考帧3D点
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt );// 当前帧2D点
    }
    
    // 确保有足够的点进行PnP计算（至少4个点）
    // Ensure we have enough points for PnP
    if (pts3d.size() < 4) {
        cout << "Not enough points for PnP (" << pts3d.size() << " < 4), skipping pose estimation" << endl;
        // 点数不足时，设置为单位变换
        // Set to identity transform when we can't estimate
        T_c_r_estimated_ = Sophus::SE3d();
        num_inliers_ = 0;
        return;
    }
    
    // 构造相机内参矩阵K
    Mat K = ( cv::Mat_<double> ( 3,3 ) <<
              ref_->camera_->fx_, 0, ref_->camera_->cx_,
              0, ref_->camera_->fy_, ref_->camera_->cy_,
              0,0,1
            );
    
    // PnP变量声明
    Mat rvec, tvec, inliers;  // 旋转向量、平移向量、内点索引
    
    // 使用RANSAC求解PnP问题
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    
    // 保存内点数量
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    
    // 将旋转向量转换为旋转矩阵
    // Convert rotation vector to rotation matrix
    Mat R;
    cv::Rodrigues(rvec, R);
    
    // 从cv::Mat创建Eigen矩阵
    // Create Eigen matrix from cv::Mat
    Eigen::Matrix3d R_eigen;
    R_eigen << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
               R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
               R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
               
    // 构造估计的位姿变换（当前帧相对于参考帧）
    T_c_r_estimated_ = Sophus::SE3d(
        Sophus::SO3d(R_eigen), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
}

/**
 * @brief 检查估计的位姿是否有效
 * @return 位姿是否有效
 */
bool VisualOdometry::checkEstimatedPose()
{
    // 检查内点数量是否足够
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    
    // 检查运动是否过大（可能是错误的估计）
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();  // 将SE3转换为李代数
    if ( d.norm() > 5.0 )  // 运动过大阈值设为5.0
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    
    return true;  // 位姿有效
}

/**
 * @brief 检查当前帧是否应作为关键帧
 * @return 是否为关键帧
 */
bool VisualOdometry::checkKeyFrame()
{
    // 获取相对运动的李代数表示
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();  // 平移部分
    Vector3d rot = d.tail<3>();    // 旋转部分
    
    // 如果旋转或平移超过设定阈值，则认为是关键帧
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    
    return false;  // 不是关键帧
}

/**
 * @brief 添加关键帧到地图
 */
void VisualOdometry::addKeyFrame()
{
    cout<<"adding a key-frame"<<endl;     // 输出提示信息
    map_->insertKeyFrame ( curr_ );       // 将当前帧插入地图
}

}