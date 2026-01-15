/**
 * @file g2o_types.h
 * @brief g2o优化中使用的自定义顶点和边
 * 
 * g2o是一个通用图优化库，本文件定义了用于视觉里程计的自定义边类型：
 * 
 * 1. EdgeProjectXYZRGBD: 3D-3D投影边（ICP）
 *    - 同时优化位姿和3D点位置
 *    
 * 2. EdgeProjectXYZRGBDPoseOnly: 仅优化位姿的3D-3D边
 *    - 固定3D点，只优化相机位姿
 *    
 * 3. EdgeProjectXYZ2UVPoseOnly: 3D-2D投影边（PnP）
 *    - 将3D点投影到2D像素，仅优化位姿
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"
#include "camera.h"

// g2o图优化库头文件
#include <g2o/core/base_vertex.h>          // 顶点基类
#include <g2o/core/base_unary_edge.h>      // 一元边基类
#include <g2o/core/block_solver.h>         // 块求解器
#include <g2o/core/optimization_algorithm_levenberg.h>  // LM算法
#include <g2o/types/sba/types_six_dof_expmap.h>  // SE3位姿顶点和3D点顶点
#include <g2o/solvers/dense/linear_solver_dense.h>  // 稠密线性求解器
#include <g2o/core/robust_kernel.h>        // 鲁棒核函数
#include <g2o/core/robust_kernel_impl.h>   // 鲁棒核函数实现

namespace myslam
{

/**
 * @class EdgeProjectXYZRGBD
 * @brief 3D-3D投影边，用于ICP优化
 * 
 * 二元边：连接3D点顶点和位姿顶点
 * 误差维度：3维（3D点坐标差异）
 * 测量值：3D点的观测坐标
 * 
 * 误差计算：e = measurement - T * point
 */
class EdgeProjectXYZRGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Eigen内存对齐宏
    
    /**
     * @brief 计算误差
     * 误差 = 测量值 - 位姿变换后的点
     */
    virtual void computeError();
    
    /**
     * @brief 计算雅可比矩阵
     * 解析求导，提高优化效率
     */
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){return true;}
    virtual bool write( std::ostream& out) const {return true;}
    
};

/**
 * @class EdgeProjectXYZRGBDPoseOnly
 * @brief 仅优化位姿的3D-3D投影边
 * 
 * 一元边：只连接位姿顶点，3D点作为固定参数
 * 误差维度：3维
 * 
 * 适用场景：已知3D点位置，仅估计相机位姿
 */
class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief 计算误差
     * 误差 = 测量值 - R*point - t
     */
    virtual void computeError();
    
    /**
     * @brief 计算雅可比矩阵
     */
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
    
    Vector3d point_;  // 3D点坐标（固定参数）
};

/**
 * @class EdgeProjectXYZ2UVPoseOnly
 * @brief 3D-2D投影边，用于PnP优化（仅优化位姿）
 * 
 * 一元边：只连接位姿顶点
 * 误差维度：2维（像素坐标差异）
 * 
 * 将3D世界点投影到图像平面，计算与观测像素的误差
 * 这是视觉里程计中最常用的边类型
 * 
 * 误差计算：e = measurement - project(T * point)
 */
class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief 计算误差
     * 误差 = 观测像素坐标 - 投影像素坐标
     */
    virtual void computeError();
    
    /**
     * @brief 计算雅可比矩阵
     * 解析求导：d(pixel)/d(pose) = d(pixel)/d(p_cam) * d(p_cam)/d(pose)
     */
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};
    
    Vector3d point_;   // 3D点坐标（固定参数）
    Camera* camera_;   // 相机模型指针，用于投影计算
};

}


#endif // MYSLAM_G2O_TYPES_H
