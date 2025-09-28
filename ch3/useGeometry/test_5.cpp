#include <iostream>
#include <Eigen/Dense>

int main() {
    // 定义一个较大的矩阵，假设大小为 5x5
    Eigen::MatrixXd largeMatrix(5, 5);

    // 初始化矩阵（这里用随机数填充，实际应用中可以按需初始化）
    largeMatrix << 1, 2, 3, 4, 5,
                   6, 7, 8, 9, 10,
                   11, 12, 13, 14, 15,
                   16, 17, 18, 19, 20,
                   21, 22, 23, 24, 25;

    std::cout << "原始矩阵:\n" << largeMatrix << std::endl;

    // 取出左上角 3x3 的子块
    Eigen::MatrixXd topLeftBlock = largeMatrix.block(0, 0, 3, 3);

    // 将子块赋值为 3x3 的单位矩阵
    topLeftBlock.setIdentity();

    // 将修改后的子块赋值回原矩阵
    largeMatrix.block(0, 0, 3, 3) = topLeftBlock;

    std::cout << "修改后的矩阵:\n" << largeMatrix << std::endl;

    return 0;
}

