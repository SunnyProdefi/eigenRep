#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

// 函数来更新障碍项参数
VectorXd updateMu(VectorXd &mu, double shrink_factor) {
  return mu * shrink_factor; // 减少障碍参数
}

// 用内点法求解二次规划问题
VectorXd interiorPointMethod(const MatrixXd &Q, const VectorXd &c,
                             const MatrixXd &A, const VectorXd &b, VectorXd x,
                             VectorXd mu, double tolerance, int maxIter) {
  const int n = x.size();           // 变量数量
  const double shrink_factor = 0.1; // 障碍参数缩减因子
  const double alpha = 0.01;        // 步长

  for (int iter = 0; iter < maxIter; ++iter) {
    VectorXd f = A * x - b; // 约束的违反量
    std::cout << "Iteration " << iter << ": f = " << f.transpose() << std::endl;
    std::cout << "f.maxCoeff() = " << f.maxCoeff() << std::endl;
    // 判断是否所有约束均已满足
    if (f.cwiseAbs().maxCoeff() < tolerance) {
      std::cout << "Solution found." << std::endl;
      break;
    }

    // 计算梯度和Hessian矩阵
    VectorXd gradient = c - A.transpose() * (mu.cwiseQuotient(f));
    MatrixXd Hessian = Q;

    // 解牛顿方程
    VectorXd deltaX = Hessian.ldlt().solve(-gradient);
    x += alpha * deltaX; // 更新解

    // 更新障碍参数
    mu = updateMu(mu, shrink_factor);

    // 输出当前迭代的结果
    std::cout << "Iteration " << iter << ": x = " << x.transpose() << std::endl;
  }

  return x;
}

int main() {
  // 定义QP问题的参数
  MatrixXd Q(2, 2);
  Q << 2, 0, 0, 2;
  VectorXd c(2);
  c << -4, -4;
  MatrixXd A(2, 2);
  A << 1, 1, -1, 2;
  VectorXd b(2);
  b << 1, 2;

  // 初始解和障碍参数
  VectorXd x(2);
  x << 0.4, 0.5; // 初始猜测
  VectorXd mu(2);
  mu << 1, 1; // 初始障碍参数

  // 调用内点法求解
  VectorXd result = interiorPointMethod(Q, c, A, b, x, mu, 0.01, 10000);

  std::cout << "Optimal solution: " << result.transpose() << std::endl;

  return 0;
}
