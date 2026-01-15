/**
 * @file frame.cpp
 * @brief 帧类的实现
 * 
 * 实现帧的创建、深度查询和视野判断等功能
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#include "myslam/frame.h"

namespace myslam
{
    using Sophus::SE3d;
    
/**
 * @brief 默认构造函数
 * 
 * 初始化帧的默认值：ID为-1，时间戳为-1，非关键帧
 */
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

/**
 * @brief 完整构造函数
 * 
 * @param id 帧ID
 * @param time_stamp 时间戳
 * @param T_c_w 相机位姿（世界到相机的变换）
 * @param camera 相机模型
 * @param color RGB彩色图像
 * @param depth 深度图像
 */
Frame::Frame ( long id, double time_stamp, Sophus::SE3d T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
{

}

/**
 * @brief 析构函数
 */
Frame::~Frame()
{

}

/**
 * @brief 工厂方法，创建新帧
 * 
 * 使用静态变量自动生成递增的帧ID
 * 
 * @return 新帧的智能指针
 */
Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;  // 静态计数器，确保每帧ID唯一
    return Frame::Ptr( new Frame(factory_id++) );
}

/**
 * @brief 获取特征点位置的深度值
 * 
 * 从深度图中查询关键点位置的深度值。
 * 如果该位置深度为0（无效），则检查上下左右4个相邻像素。
 * 
 * @param kp 关键点
 * @return 深度值（米），如果无有效深度返回-1.0
 */
double Frame::findDepth ( const cv::KeyPoint& kp )
{
    // 将浮点坐标四舍五入为整数像素坐标
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    
    // 读取深度值（深度图通常为16位无符号整数，单位为毫米）
    ushort d = depth_.ptr<ushort>(y)[x];
    
    if ( d!=0 )
    {
        // 有效深度：转换为米
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // 深度无效，检查4邻域像素
        // dx, dy定义了上下左右四个方向
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    // 找不到有效深度
    return -1.0;
}

/**
 * @brief 设置帧的位姿
 * 
 * @param T_c_w 新的相机位姿（世界到相机的变换）
 */
void Frame::setPose ( const Sophus::SE3d& T_c_w )
{
    T_c_w_ = T_c_w;
}

/**
 * @brief 获取相机中心在世界坐标系中的位置
 * 
 * 相机中心即光心位置，等于T_c_w逆变换的平移部分
 * T_w_c = T_c_w^(-1), 相机中心 = T_w_c.translation()
 * 
 * @return 相机光心的世界坐标
 */
Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

/**
 * @brief 判断世界坐标点是否在当前帧的视野内
 * 
 * 判断条件：
 * 1. 点在相机前方（相机坐标系Z > 0）
 * 2. 投影后的像素坐标在图像范围内
 * 
 * @param pt_world 世界坐标系下的3D点
 * @return 是否在视野内
 */
bool Frame::isInFrame ( const Vector3d& pt_world )
{
    // 转换到相机坐标系
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    
    // 检查是否在相机前方
    if ( p_cam(2,0)<0 ) return false;
    
    // 投影到像素坐标
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    
    // 检查像素坐标是否在图像范围内
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}

}
