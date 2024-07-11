#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

int main() {
  // 定义一个3x3的旋转矩阵
  Eigen::Matrix3d R;

  // 初始化矩阵的元素
  R << 0.7071068, -0.7071068, 0.0000000, 0.5000000, 0.5000000, -0.7071068,
      0.5000000, 0.5000000, 0.7071068;

  // 打印矩阵R
  std::cout << "Matrix R:\n" << R << std::endl;

  // 从旋转矩阵R构建AngleAxis对象
  Eigen::AngleAxisd angleAxis(R);

  // 提取旋转角度和旋转轴
  double angle = angleAxis.angle();
  Eigen::Vector3d axis = angleAxis.axis();

  angle = 180.0 / M_PI * angle; // 将角度转换为度
  // 输出旋转角度和旋转轴
  std::cout << "Rotation angle (radians): " << angle << std::endl;
  std::cout << "Rotation axis: " << axis.transpose() << std::endl;

  return 0;
}
