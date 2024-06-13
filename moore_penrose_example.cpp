#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

int main() {
  // 定义矩阵 A 和向量 b
  MatrixXd A(3, 2);
  A << 1, 2, 3, 4, 5, 6;
  VectorXd b(3);
  b << 7, 8, 10;

  // 计算广义逆
  MatrixXd A_pinv = A.completeOrthogonalDecomposition().pseudoInverse();
  //最小二乘解
  VectorXd x = A_pinv * b;
  // 输出结果
  cout << "A^+ = \n" << A_pinv << endl;
  cout << "最小二乘解 x = \n" << x << endl;

  // 验证 Ax ≈ b
  VectorXd b_approx = A * x;
  cout << "Ax ≈ \n" << b_approx << endl;

  return 0;
}
