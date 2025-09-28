#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

using namespace std;

int main(int argc, char** argv) {
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // 构造SO3对象
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_v = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, M_PI / 2));
    Eigen::Quaterniond q(R);
    Sophus::SO3d SO3_q = Sophus::SO3d::exp(q.coeffs().head<3>());

    // 输出 SO3 矩阵
    cout << "SO(3) from matrix: " << SO3_R.matrix() << endl;

    // 修复：直接获取李代数向量（log 返回 Vector3d）
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;

    // 修复：hat 生成反对称矩阵，vee 还原为向量
    cout << "so3 hat =\n" << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    // 增量更新 SO3
    Eigen::Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = " << SO3_updated.matrix() << endl;

    cout << "************我是分割线*************" << endl;

    // 构造 SE3 对象
    Eigen::Vector3d t(1, 0, 0);
    Sophus::SE3d SE3_Rt(SO3_R, t); // 直接构造 SE3
    Sophus::SE3d SE3_qt(SO3_q, t);

    // 修复：直接获取 SE3 的李代数向量（log 返回 Vector6d）
    Sophus::Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;

    // 修复：hat 生成 4x4 矩阵，vee 还原为 Vector6d
    cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    // SE3 更新
    Sophus::Vector6d update_se3 = Sophus::Vector6d::Zero();
    update_se3(0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = \n" << SE3_updated.matrix() << endl;

    return 0;
}