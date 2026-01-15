# g2o_types.cpp 数学模型详细解释
该文件实现了SLAM系统中用于优化的几种关键边类型，主要涉及3D点与相机位姿之间的投影关系。以下是详细数学模型解释：

## 1. EdgeProjectXYZRGBD (RGB-D相机点云边)
### 误差函数
- **数学模型**：$e = z_{obs} - T_{wc} \cdot P_w$    
    - $z_{obs}$: 深度相机观测到的3D点坐标（测量值）
    - $T_{wc}$: 相机位姿（世界坐标系到相机坐标系的变换）
    - $P_w$: 世界坐标系下的3D点坐标
- **代码实现**：`_error = _measurement - pose->estimate().map(point->estimate())`
- _measurement 对应 $z_{obs}$
- pose->estimate().map(point->estimate()) 对应 $T_{wc} \cdot P_w$
### 雅可比矩阵
- **对3D点的导数**（_jacobianOplusXi，3×3矩阵）：

    - $\frac{\partial e}{\partial P_w} = -R_{wc}$（旋转矩阵）
    - 实际计算中简化为 $-I_{3\times3}$，因为点坐标在自身坐标系下
- **对位姿的导数**（_jacobianOplusXj，3×6矩阵）：
    - $\frac{\partial e}{\partial \xi} = -[I_{3\times3} \mid -(P_c)_\times]$
    - 其中 $P_c = [x, y, z]^T$ 是相机坐标系下的3D点
    - $(P_c)_\times$ 是 $P_c$ 的反对称矩阵
    - 矩阵形式：
```
0, -z,  y, -1,  0,  0,
z,  0, -x,  0, -1,  0,
-y, x,  0,  0,  0, -1
```

## 2. EdgeProjectXYZRGBDPoseOnly (仅优化位姿的RGB-D边)
### 误差函数
- **数学模型**：$e = z_{obs} - T_{wc} \cdot P_w$
    - 与EdgeProjectXYZRGBD相同，但 $P_w$ 是固定值（不优化）
- 代码实现：_error = _measurement - pose->estimate().map(point_)
    - point_ 是类成员变量，存储固定的3D点坐标
### 雅可比矩阵
- 仅对位姿的导数（_jacobianOplusXi，3×6矩阵）：
    - 与EdgeProjectXYZRGBD中对位姿的雅可比矩阵完全相同
    - $\frac{\partial e}{\partial \xi} = -[I_{3\times3} \mid -(P_c)_\times]$
    - 用于仅优化相机位姿，而3D点坐标保持固定的情况

## 3. EdgeProjectXYZ2UVPoseOnly (单目相机投影边)
### 误差函数
- **数学模型**：$e = [u_{obs}, v_{obs}]^T - \pi(T_{wc} \cdot P_w)$
    - $\pi(\cdot)$: 相机投影函数
    - 投影公式：$u = f_x \cdot \frac{x}{z} + c_x$，$v = f_y \cdot \frac{y}{z} + c_y$
    - $f_x, f_y$: 相机焦距
    - $c_x, c_y$: 主点坐标
- **代码实现**：
```cpp
_error = obs - camera_->camera2pixel(point_camera);
```
- `camera2pixel` 实现了上述投影函数
## 雅可比矩阵
- **对位姿的导数**（_jacobianOplusXi，2×6矩阵）：
    - 通过链式法则计算：$\frac{\partial e}{\partial \xi} = \frac{\partial e}{\partial P_c} \cdot \frac{\partial P_c}{\partial \xi}$
    - 其中 $\frac{\partial P_c}{\partial \xi} = -[I_{3\times3} \mid -(P_c)_\times]$
- **投影函数对3D点的导数**：
    - $\frac{\partial u}{\partial x} = \frac{f_x}{z}$，$\frac{\partial u}{\partial y} = 0$，$\frac{\partial u}{\partial z} = -\frac{f_x x}{z^2}$
    - $\frac{\partial v}{\partial x} = 0$，$\frac{\partial v}{\partial y} = \frac{f_y}{z}$，$\frac{\partial v}{\partial z} = -\frac{f_y y}{z^2}$
- 最终雅可比矩阵：
    - u坐标行：
        - 旋转部分：$[\frac{xy}{z^2}f_x, -(1+\frac{x^2}{z^2})f_x, \frac{y}{z}f_x]$
        - 平移部分：$[-\frac{1}{z}f_x, 0, \frac{x}{z^2}f_x]$
    - v坐标行：
        - 旋转部分：$[(1+\frac{y^2}{z^2})f_y, -\frac{xy}{z^2}f_y, -\frac{x}{z}f_y]$
        - 平移部分：$[0, -\frac{1}{z}f_y, \frac{y}{z^2}f_y]$

这些边类型构成了SLAM系统中后端优化的核心，通过最小化这些误差项，系统可以同时优化相机轨迹和地图点位置，实现精确的定位与建图。