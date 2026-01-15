/**
 * @file common_include.h
 * @brief 公共头文件 - 包含项目中常用的库和类型定义
 * 
 * 本文件集中管理项目中频繁使用的第三方库头文件，包括：
 * - Eigen: 用于矩阵运算和几何变换
 * - Sophus: 用于李群和李代数运算（SE3, SO3）
 * - OpenCV: 用于图像处理
 * - STL: 标准模板库容器
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// ==================== Eigen 矩阵库 ====================
// Eigen是一个高性能的C++线性代数库，用于矩阵和向量运算
#include <Eigen/Core>       // 核心矩阵运算
#include <Eigen/Geometry>   // 几何变换（旋转、平移等）
using Eigen::Vector2d;      // 2维向量（像素坐标等）
using Eigen::Vector3d;      // 3维向量（3D点坐标等）

// ==================== Sophus 李群库 ====================
// Sophus库提供了李群（SE3, SO3）的C++实现，用于表示刚体变换
#include <sophus/se3.hpp>   // SE3: 特殊欧氏群，表示3D空间的刚体变换（旋转+平移）
#include <sophus/so3.hpp>   // SO3: 特殊正交群，表示3D空间的旋转
using Sophus::SO3;          // SO3类型
using Sophus::SE3;          // SE3类型
typedef Sophus::SE3<double> SE3d;  // 双精度SE3类型别名
typedef Sophus::SO3<double> SO3d;  // 双精度SO3类型别名

// ==================== OpenCV 图像库 ====================
// OpenCV用于图像处理、特征提取和匹配
#include <opencv2/core/core.hpp>
using cv::Mat;              // OpenCV矩阵类型，用于存储图像

// ==================== C++ 标准库 ====================
#include <vector>           // 动态数组容器
#include <list>             // 双向链表容器
#include <memory>           // 智能指针（shared_ptr等）
#include <string>           // 字符串类
#include <iostream>         // 输入输出流
#include <set>              // 集合容器
#include <unordered_map>    // 哈希表容器
#include <map>              // 有序关联容器

using namespace std; 
#endif