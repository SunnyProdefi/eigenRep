#include <Eigen/Dense>
#include <iostream>

using namespace std;

int main() {
    // 创建一个4x4的动态大小矩阵并初始化
    Eigen::MatrixXf m(4,4);
    m <<  1, 2, 3, 4,
          5, 6, 7, 8,
          9, 10, 11, 12,
          13, 14, 15, 16;

    // 打印矩阵中间的2x2块
    cout << "Block in the middle:" << endl;
    cout << m.block<2,2>(1,1) << endl << endl;

    // 修改矩阵中间的2x2块
    m.block<2,2>(1,1) *= 2;
    cout << "After modification:" << endl;
    cout << m << endl;
}
    