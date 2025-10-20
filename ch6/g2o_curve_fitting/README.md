# g2o 笔记

``` shell
iteration= 0     chi2= 30373.727656      time= 7.0808e-05        cumTime= 7.0808e-05     edges= 100      schur= 0        lambda= 699.050482      levenbergIter= 7
iteration= 1     chi2= 13336.948287      time= 3.727e-05         cumTime= 0.000108078    edges= 100      schur= 0        lambda= 1864.134619     levenbergIter= 3
iteration= 2     chi2= 6946.262238       time= 3.1722e-05        cumTime= 0.0001398      edges= 100      schur= 0        lambda= 1242.756412     levenbergIter= 1
iteration= 3     chi2= 271.023143        time= 3.1781e-05        cumTime= 0.000171581    edges= 100      schur= 0        lambda= 414.252137      levenbergIter= 1
iteration= 4     chi2= 118.903888        time= 3.0441e-05        cumTime= 0.000202022    edges= 100      schur= 0        lambda= 138.084046      levenbergIter= 1
iteration= 5     chi2= 113.568661        time= 3.2233e-05        cumTime= 0.000234255    edges= 100      schur= 0        lambda= 46.028015       levenbergIter= 1
iteration= 6     chi2= 107.476468        time= 3.1657e-05        cumTime= 0.000265912    edges= 100      schur= 0        lambda= 15.342672       levenbergIter= 1
iteration= 7     chi2= 103.014521        time= 3.1431e-05        cumTime= 0.000297343    edges= 100      schur= 0        lambda= 5.114224        levenbergIter= 1
iteration= 8     chi2= 101.988349        time= 3.0391e-05        cumTime= 0.000327734    edges= 100      schur= 0        lambda= 1.704741        levenbergIter= 1
iteration= 9     chi2= 101.937388        time= 3.1767e-05        cumTime= 0.000359501    edges= 100      schur= 0        lambda= 0.568247        levenbergIter= 1
......
```

g2o优化迭代参数详解
1. iteration（迭代次数）
表示当前是第几次迭代，从0开始计数。优化过程会持续多轮迭代，直到收敛或达到最大迭代次数。

2. chi2（卡方值）
这是目标函数的值，也就是所有边的误差平方和：

chi2 = Σ(error_i²)
初始值为30373.727656，经过10次迭代后下降到101.937388
这个值越小表示拟合效果越好
可以看到优化过程快速收敛，前几次迭代下降明显
3. time（单次迭代时间）
表示本次迭代所用的时间，以秒为单位。例如3.727e-05表示0.00003727秒。

4. cumTime（累计时间）
表示从开始优化到当前迭代结束所用的总时间。

5. edges（边的数量）
表示图中边的数量，这里是100，对应100个数据点，每个点对应一条边（一个误差项）。

6. schur（Schur补标志）
表示是否使用了Schur补技巧来加速计算。0表示未使用，1表示使用。这里因为问题规模较小，未使用Schur补。

7. lambda（阻尼因子）
这是Levenberg-Marquardt算法中的关键参数：

初始值为699.050482，随着迭代进行逐渐减小到0.568247
当lambda较大时，算法更接近梯度下降法，适合在远离最优解时使用
当lambda较小时，算法更接近高斯牛顿法，适合在接近最优解时使用
g2o会根据误差下降情况自动调整lambda值
8. levenbergIter（Levenberg迭代次数）
表示在当前迭代中，为找到合适的步长而进行的Levenberg-Marquardt子迭代次数：

大部分情况下为1，说明每次主迭代只需要一次子迭代就能找到合适的步长
第一次迭代用了7次子迭代，说明初始阶段调整较为困难
优化过程分析
从输出可以看出：

快速收敛：前几次迭代chi2值下降很快（从30373降到271），说明优化方向正确
后期精细调整：后几次迭代chi2值下降缓慢（从271降到101），说明在进行精细调整
算法自适应性：lambda值从699逐渐减小到0.568，体现了Levenberg-Marquardt算法的自适应特性
高效性：每次迭代时间很短，说明算法效率很高
Levenberg-Marquardt算法原理
Levenberg-Marquardt算法通过以下公式更新参数：

(H + λI)Δx = -g
x_{k+1} = x_k + Δx
其中：

H是Hessian矩阵
I是单位矩阵
λ是阻尼因子
g是梯度向量
算法根据误差下降情况动态调整λ：

如果误差下降明显，减小λ，更接近高斯牛顿法
如果误差下降不明显，增大λ，更接近梯度下降法
这就是为什么您看到的输出中lambda值逐渐减小，而每次迭代的子迭代次数(levenbergIter)大多为1的原因。


# 优化问题的数学建模
1. 问题定义
这个程序解决的是一个非线性最小二乘曲线拟合问题。目标是找到一个指数函数的参数，使其最好地拟合给定的数据点。

具体来说，我们希望拟合的函数形式为：
```
y = exp(a*x² + b*x + c)
```
其中 a, b, c 是我们需要优化的参数。

2. 数学模型
这是一个典型的非线性最小二乘问题，其数学形式为：
```
minimize: F(a,b,c) = 1/2 * Σ(i=1 to N) [yi - exp(a*xi² + b*xi + c)]²
```
其中：

N 是数据点的数量（这里是100个）
(xi, yi) 是第i个观测数据点
我们的目标是找到参数 a, b, c 使得所有观测点与模型预测值之间的误差平方和最小
3. 转换为图优化问题
在g2o框架中，这个问题被建模为图优化问题：

顶点(Vertex)
优化变量：参数向量 [a, b, c]，维度为3
用CurveFittingVertex类表示，继承自g2o::BaseVertex<3, Eigen::Vector3d>
边(Edge)
误差项：每个数据点(xi, yi)对应一个误差项
误差计算公式：error = yi - exp(a*xi² + b*xi + c)
用CurveFittingEdge类表示，继承自g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
4. 优化过程
线性化
由于这是非线性优化问题，g2o使用迭代方法求解。在每次迭代中，将非线性误差函数在当前点附近进行一阶泰勒展开：
```
error(xi) ≈ error(x0) + J(x0) * Δx
```
其中：

J(x0) 是雅可比矩阵（误差函数对参数的偏导数）
Δx 是参数的增量
构建线性系统
通过线性化，原问题转换为每次迭代求解如下线性系统：
```
H * Δx = b
```
其中：

H 是Hessian矩阵（由雅可比矩阵构成）
b 是梯度向量
Δx 是参数更新量
求解方法
程序使用Levenberg-Marquardt算法求解，它在高斯牛顿法和梯度下降法之间自适应调整：
```
(H + λI) * Δx = b
```
其中λ是阻尼因子，I是单位矩阵。

5. 雅可比矩阵计算
对于每个误差项，我们需要计算误差对参数的偏导数：

J = [∂error/∂a, ∂error/∂b, ∂error/∂c]
具体计算：
```
∂error/∂a = ∂/∂a [y - exp(a*x² + b*x + c)] = -x² * exp(a*x² + b*x + c)
∂error/∂b = ∂/∂b [y - exp(a*x² + b*x + c)] = -x * exp(a*x² + b*x + c)
∂error/∂c = ∂/∂c [y - exp(a*x² + b*x + c)] = -exp(a*x² + b*x + c)
```
这些导数在g2o内部自动计算（通过自动求导或数值求导）。

6. 权重和信息矩阵
程序中使用了信息矩阵（协方差矩阵的逆）来表示观测的不确定性：

information_matrix = I * 1/(σ²)
其中σ是噪声的标准差，I是单位矩阵。这使得更可信的观测值在优化中具有更大的权重。

7. 整体优化流程
初始化参数 [a, b, c] = [0, 0, 0]
对每个数据点计算误差和雅可比矩阵
构建整体的线性系统 HΔx = b
使用Levenberg-Marquardt算法求解参数更新量 Δx
更新参数：[a, b, c] = [a, b, c] + Δx
重复步骤2-5直到收敛或达到最大迭代次数
