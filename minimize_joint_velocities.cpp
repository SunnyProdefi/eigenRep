#include <Eigen/Dense>
#include <iostream>

int main() {
  // 假设我们有一个3x3的雅可比矩阵 J_w 和期望速度向量 v_d
  Eigen::MatrixXd J_w(3, 3);
  J_w << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  Eigen::VectorXd v_d(3);
  v_d << 1, 1, 1;

  // 使用广义逆求解
  Eigen::VectorXd dot_q = J_w.completeOrthogonalDecomposition().solve(v_d);

  std::cout << "Optimal joint velocities (dot_q):\n" << dot_q << std::endl;

  return 0;
}
