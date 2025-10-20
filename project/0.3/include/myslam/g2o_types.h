/*
 * <one line to give the program's name and a brief idea of what it does.>
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

#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"
#include "camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace myslam
{
/**
 * RGBD边类，用于同时优化3D点和位姿
 * 继承自g2o::BaseBinaryEdge，表示二元边
 * 模板参数含义：
 * - 3: 误差项维度（x,y,z三个坐标）
 * - Eigen::Vector3d: 测量值类型（3D点坐标）
 * - g2o::VertexPointXYZ: 第一个顶点类型（3D点）
 * - g2o::VertexSE3Expmap: 第二个顶点类型（位姿）
 */
class EdgeProjectXYZRGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    /**
     * 计算误差函数
     * 误差 = 测量值 - 通过位姿变换后的3D点坐标
     */
    virtual void computeError();
    
    /**
     * 线性化函数，计算雅可比矩阵
     * 计算误差函数对顶点的偏导数
     */
    virtual void linearizeOplus();
    
    /**
     * 从输入流读取数据
     * @param in 输入流
     * @return 是否读取成功
     */
    virtual bool read( std::istream& in ){return true;}
    
    /**
     * 向输出流写入数据
     * @param out 输出流
     * @return 是否写入成功
     */
    virtual bool write( std::ostream& out) const {return true;}
    
};

/**
 * RGBD位姿优化边类，仅优化位姿，不优化3D点
 * 继承自g2o::BaseUnaryEdge，表示一元边
 * 模板参数含义：
 * - 3: 误差项维度（x,y,z三个坐标）
 * - Eigen::Vector3d: 测量值类型（3D点坐标）
 * - g2o::VertexSE3Expmap: 顶点类型（位姿）
 */
class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * 计算误差函数
     * 误差 = 测量值 - 通过位姿变换后的固定3D点坐标
     */
    virtual void computeError();
    
    /**
     * 线性化函数，计算雅可比矩阵
     * 计算误差函数对位姿顶点的偏导数
     */
    virtual void linearizeOplus();
    
    /**
     * 从输入流读取数据
     * @param in 输入流
     * @return 是否读取成功
     */
    virtual bool read( std::istream& in ){return true;}
    
    /**
     * 向输出流写入数据
     * @param out 输出流
     * @return 是否写入成功
     */
    virtual bool write( std::ostream& out) const {return true;}
    
    Vector3d point_;  // 固定的3D点坐标
};

/**
 * 2D-3D点投影边类，用于单目相机位姿优化
 * 继承自g2o::BaseUnaryEdge，表示一元边
 * 模板参数含义：
 * - 2: 误差项维度（u,v两个像素坐标）
 * - Eigen::Vector2d: 测量值类型（2D像素坐标）
 * - g2o::VertexSE3Expmap: 顶点类型（位姿）
 */
class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * 计算误差函数
     * 误差 = 测量像素值 - 3D点通过位姿和相机内参投影到图像平面的坐标
     */
    virtual void computeError();
    
    /**
     * 线性化函数，计算雅可比矩阵
     * 计算误差函数对位姿顶点的偏导数
     */
    virtual void linearizeOplus();
    
    /**
     * 从输入流读取数据
     * @param in 输入流
     * @return 是否读取成功
     */
    virtual bool read( std::istream& in ){return true;}
    
    /**
     * 向输出流写入数据
     * @param os 输出流
     * @return 是否写入成功
     */
    virtual bool write(std::ostream& os) const {return true;}
    
    Vector3d point_;    // 固定的3D点世界坐标
    Camera* camera_;    // 相机模型指针，用于坐标变换
};

}


#endif // MYSLAM_G2O_TYPES_H