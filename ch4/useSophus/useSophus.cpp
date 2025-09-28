#include <iostream>
#include <cmath>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv) {
    cout << "Sophus test" << endl;
    cout.precision(3);

    // Rotation matrix for 90 degrees around Z-axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    
    // Valid constructors
    Sophus::SO3<double> SO3_R(R);               // From rotation matrix
    // Sophus::SO3<double> SO3_v(0.0, 0.0, M_PI/2); // From Euler angles (use double)
    Eigen::Quaterniond q(R);                     // From quaternion
    Sophus::SO3<double> SO3_q(q);

    // Output with explicit conversion
    cout << "SO(3) from matrix: " << endl << SO3_R.matrix() << endl;
    // cout << "SO(3) from vector: " << SO3_v.log() << endl;
    cout << "SO(3) from quaternion: " << endl << SO3_q.matrix() << endl;

    // Lie algebra operations
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
    // cout << "so3 hat vee= " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

    return 0;
}