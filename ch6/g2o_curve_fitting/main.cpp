#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <memory>
#include <chrono>
using namespace std; 

/**
 * @brief 曲线拟合顶点类
 * 
 * 该类用于表示曲线拟合问题中的优化变量顶点，继承自g2o::BaseVertex
 * 模板参数：
 * - 3: 优化变量的维度，这里表示曲线方程 y = exp(ax^2 + bx + c) 中的三个参数 a, b, c
 * - Eigen::Vector3d: 顶点存储的数据类型，使用3维向量存储参数 [a, b, c]
 */
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief 重置顶点参数值
     * 
     * 将顶点的估计值重置为零向量 [0, 0, 0]
     * 在优化开始前调用此函数将顶点值初始化为默认值
     */
    virtual void setToOriginImpl() // 重置
    {
        _estimate << 0,0,0;
    }
    
    /**
     * @brief 更新顶点参数值
     * 
     * 根据输入的更新量 update 更新顶点的估计值
     * 这是图优化中顶点更新的关键步骤
     * 
     * @param update 更新量，指向包含3个double值的数组，分别对应 a, b, c 的更新量
     */
    virtual void oplusImpl( const double* update ) // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }
    
    /**
     * @brief 从流中读取顶点数据
     * 
     * 实现顶点数据的持久化读取功能，这里留空表示不实现
     * 
     * @param in 输入流
     * @return bool 读取是否成功
     */
    virtual bool read( istream& in ) override { return false; }
    
    /**
     * @brief 向流中写入顶点数据
     * 
     * 实现顶点数据的持久化保存功能，这里留空表示不实现
     * 
     * @param out 输出流
     * @return bool 写入是否成功
     */
    virtual bool write( ostream& out ) const override { return false; }
};

/**
 * @brief 曲线拟合边类
 * 
 * 该类用于表示曲线拟合问题中的边（误差项），继承自g2o::BaseUnaryEdge
 * 模板参数：
 * - 1: 观测值维度，这里表示一个标量误差值
 * - double: 观测值的数据类型
 * - CurveFittingVertex: 连接的顶点类型
 */
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief 构造函数
     * 
     * 初始化边并设置对应的x坐标值
     * 
     * @param x x坐标值
     */
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    
    /**
     * @brief 计算曲线拟合误差
     * 
     * 计算实际观测值与模型预测值之间的误差
     * 模型方程为: y = exp(ax^2 + bx + c)
     * 误差定义为: error = measurement - exp(ax^2 + bx + c)
     */
    void computeError()
    {
        // 获取连接的顶点
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        // 获取顶点的参数估计值 [a, b, c]
        const Eigen::Vector3d abc = v->estimate();
        // 计算误差：观测值 - 模型预测值
        _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
    }
    
    /**
     * @brief 从流中读取边数据
     * 
     * 实现边数据的持久化读取功能，这里留空表示不实现
     * 
     * @param in 输入流
     * @return bool 读取是否成功
     */
    virtual bool read( istream& in ) override { return false; }
    
    /**
     * @brief 向流中写入边数据
     * 
     * 实现边数据的持久化保存功能，这里留空表示不实现
     * 
     * @param out 输出流
     * @return bool 写入是否成功
     */
    virtual bool write( ostream& out ) const override { return false; }
public:
    double _x;  // x 值， y 值为 _measurement
};

/**
 * @brief 主函数
 * 
 * 实现基于g2o的曲线拟合功能，拟合目标曲线为 y = exp(ax^2 + bx + c)
 * 程序流程：
 * 1. 生成带噪声的模拟数据
 * 2. 构建g2o优化器
 * 3. 添加顶点和边
 * 4. 执行优化
 * 5. 输出结果
 */
int main( int argc, char** argv )
{
    double a=1.0, b=2.0, c=1.0;         // 预设参数值，是我们待优化的参数
    int N=100;                          // 数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据
    
    cout<<"generating data: "<<endl;
    // 生成N个带噪声的数据点，基于模型 y = exp(ax^2 + bx + c) + w
    for ( int i=0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )          // 模型函数
        );
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }
    
    // 构建图优化，先设定g2o
    // 定义块求解器类型，参数为<误差项优化变量维度，误差值维度>
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;  
    // 创建线性求解器，使用稠密矩阵求解
    auto linearSolver = std::make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>(); 
    // 创建块求解器，并将线性求解器传递给它
    auto solver_ptr = std::make_unique<Block>( std::move(linearSolver) );      
    // 选择优化算法，这里使用Levenberg-Marquardt算法
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
    // 可选的其他算法：
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( std::move(solver_ptr) );
    
    // 创建稀疏优化器并设置求解器
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器
    optimizer.setVerbose( true );       // 打开调试输出
    
    // 往图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );  // 设置初始估计值
    v->setId(0);                               // 设置顶点ID
    optimizer.addVertex( v );                  // 将顶点添加到优化器
    
    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );  // 创建边
        edge->setId(i);                                              // 设置边ID
        edge->setVertex( 0, v );                                     // 设置连接的顶点
        edge->setMeasurement( y_data[i] );                           // 设置观测数值
        // 设置信息矩阵（协方差矩阵之逆），这里假设测量噪声服从高斯分布
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); 
        optimizer.addEdge( edge );                                   // 将边添加到优化器
    }
    
    // 执行优化
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();  // 记录开始时间
    optimizer.initializeOptimization();                                  // 初始化优化
    optimizer.optimize(100);                                             // 执行100次迭代优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();  // 记录结束时间
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
    
    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();                       // 获取优化后的参数估计值
    cout<<"estimated model: "<<abc_estimate.transpose()<<endl;
    
    return 0;
}


/**
 * g2o求解器架构说明
g2o的求解器架构分为三层：

最上层：优化算法

g2o::OptimizationAlgorithmLevenberg: Levenberg-Marquardt算法
g2o::OptimizationAlgorithmGaussNewton: 高斯牛顿法
g2o::OptimizationAlgorithmDogleg: Dogleg算法
中间层：块求解器

g2o::BlockSolver: 处理块稀疏结构的线性系统求解
最下层：线性求解器

g2o::LinearSolverDense: 稠密矩阵求解器
g2o::LinearSolverCholmod: 基于CHOLMOD的稀疏求解器
g2o::LinearSolverCSparse: 基于CSparse的稀疏求解器


 */