#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>

int main() {
  // 示例数据，根据实际情况调整
  Eigen::MatrixXd hessian(2, 2);
  Eigen::VectorXd gradient(2);
  Eigen::MatrixXd A_ieq(2, 2);
  Eigen::VectorXd lower_bound(2);
  Eigen::VectorXd upper_bound(2);

  // 填充Hessian矩阵和梯度向量
  hessian << 4, 1, 1, 2;
  gradient << 1, 1;

  // 线性不等式约束矩阵和界限
  A_ieq << 1, 0, 0, 1;
  lower_bound << 0, 0;
  upper_bound << 1, 1;
  // 转换矩阵到稀疏格式
  Eigen::SparseMatrix<double> sparse_hessian = hessian.sparseView();
  Eigen::SparseMatrix<double> sparse_A_ieq = A_ieq.sparseView();
  // 打印QP问题信息
  std::cout << "Standard QP problem:" << std::endl;
  std::cout << "Hessian:\n" << hessian << std::endl;
  std::cout << "Gradient:\n" << gradient << std::endl;
  std::cout << "A_ieq:\n" << A_ieq << std::endl;
  std::cout << "Lower bound:\n" << lower_bound << std::endl;
  std::cout << "Upper bound:\n" << upper_bound << std::endl;

  // 设置OSQP求解器
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(true);

  // 设置QP问题
  solver.data()->setNumberOfVariables(2);
  solver.data()->setNumberOfConstraints(2);
  if (!solver.data()->setHessianMatrix(sparse_hessian))
    return 1;
  if (!solver.data()->setGradient(gradient))
    return 1;
  if (!solver.data()->setLinearConstraintsMatrix(sparse_A_ieq))
    return 1;
  if (!solver.data()->setLowerBound(lower_bound))
    return 1;
  if (!solver.data()->setUpperBound(upper_bound))
    return 1;

  if (!solver.initSolver())
    return 1;

  // 求解QP问题
  solver.solveProblem(); // Updated to use the new API
  Eigen::VectorXd solution = solver.getSolution();
  std::cout << "Solution:" << solution << std::endl;

  return 0;
}
