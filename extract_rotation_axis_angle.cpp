#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace Eigen;

// 函数：从旋转矩阵提取旋转轴和旋转角度
void extractRotationAxisAngle(const Matrix3d &R, Vector3d &axis,
                              double &angle) {
  // 计算旋转角度
  angle = acos((R.trace() - 1) / 2.0);

  // 计算旋转轴
  if (angle > 1e-6) {
    axis(0) = R(2, 1) - R(1, 2);
    axis(1) = R(0, 2) - R(2, 0);
    axis(2) = R(1, 0) - R(0, 1);
    axis.normalize();
  } else {
    // 如果角度非常小，轴可以是任意向量，这里选择单位向量
    axis = Vector3d::UnitX();
  }
}

int main() {
  // 示例旋转矩阵（你可以替换为你自己的旋转矩阵）
  Matrix3d R;
  R = AngleAxisd(M_PI / 4, Vector3d::UnitZ()); // 绕Z轴旋转45度

  // 提取旋转轴和旋转角度
  Vector3d axis;
  double angle;
  extractRotationAxisAngle(R, axis, angle);

  // 输出结果
  std::cout << "旋转矩阵 R:\n" << R << std::endl;
  std::cout << "旋转轴:\n" << axis.transpose() << std::endl;
  std::cout << "旋转角度 (弧度):\n" << angle << std::endl;
  std::cout << "旋转角度 (度):\n" << angle * 180.0 / M_PI << std::endl;

  return 0;
}
