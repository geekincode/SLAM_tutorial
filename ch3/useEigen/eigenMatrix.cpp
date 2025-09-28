#include <iostream>
#include <ctime>

using namespace std;

// Eigen part
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char **argv)
{ 
    cout << "This is a test for Eigen!" << endl;

    Eigen::Matrix<float, 2, 3> matrix_23;
    Eigen::Vector3d v_3d;

    matrix_23 << 1, 2, 3, 4, 5, 6;
    cout << matrix_23 << endl;

    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 3; j++){
            cout << matrix_23(i, j) << "\t";
        }
        cout << endl;
    }

    v_3d << 3, 2, 1;

    Eigen::Matrix<double, 2, 1> matrix_23_1 = matrix_23.cast<double>() * v_3d;
    cout << matrix_23_1 << endl;

    auto matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;

    cout << matrix_23.transpose() << endl;
    cout << matrix_33.inverse() << endl;
    cout << matrix_33.determinant() << endl;
    cout << matrix_33.trace() << endl;

    return 0;
}
