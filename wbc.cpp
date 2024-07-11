#include <Eigen/Dense>
#include <iostream>

int main() {
  // 定义机器人的自由度
  const int num_joints = 6;

  // 模拟的任务空间速度向量（例如，末端执行器的速度和方向）
  Eigen::VectorXd task_space_velocity(6);
  task_space_velocity << 0.1, -0.1, 0.2, 0.05, 0.05, 0.1;

  // 任务权重矩阵，可以调整每个任务的优先级
  Eigen::MatrixXd task_weight_matrix = Eigen::MatrixXd::Identity(6, 6);
  task_weight_matrix(2, 2) = 2; // 提高第三个任务的权重
                                // 打印任务权重矩阵
  std::cout << "Task weight matrix:\n" << task_weight_matrix << std::endl;

  // 模拟的Jacobian矩阵，描述了关节空间到任务空间的映射
  Eigen::MatrixXd jacobian_matrix = Eigen::MatrixXd::Random(6, num_joints);

  // 计算伪逆
  Eigen::MatrixXd pseudo_inverse =
      jacobian_matrix.transpose() *
      (jacobian_matrix * jacobian_matrix.transpose() + task_weight_matrix)
          .inverse();

  // 打印伪逆矩阵
  std::cout << "Pseudo inverse:\n" << pseudo_inverse << std::endl;

  Eigen::MatrixXd pseudo_inverse1 =
      jacobian_matrix.transpose() *
      (jacobian_matrix * jacobian_matrix.transpose()).inverse();

  // 打印伪逆矩阵
  std::cout << "Pseudo inverse1:\n" << pseudo_inverse1 << std::endl;

  // 计算关节空间的速度
  Eigen::VectorXd joint_space_velocity = pseudo_inverse * task_space_velocity;

  // 输出结果
  std::cout << "Joint space velocity:\n" << joint_space_velocity << std::endl;

  return 0;
}