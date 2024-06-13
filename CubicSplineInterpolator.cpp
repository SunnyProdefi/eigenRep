#include <iostream>
#include <unsupported/Eigen/Splines>
#include <vector>

typedef Eigen::Spline<double, 3> Spline3d;

std::vector<Eigen::Vector3d>
generateSplinePoints(const std::vector<Eigen::Vector3d> &control_points,
                     int num_samples) {
  Eigen::MatrixXd points(3, control_points.size());
  for (size_t i = 0; i < control_points.size(); ++i) {
    points.col(i) = control_points[i];
  }

  // 创建三次样条插值
  auto spline =
      Eigen::SplineFitting<Spline3d>::Interpolate(points, 3); // 3 表示三次样条

  // 根据样条曲线生成点
  std::vector<Eigen::Vector3d> spline_points;
  double t_step = 1.0 / (num_samples - 1);
  for (int i = 0; i < num_samples; ++i) {
    double t = i * t_step;
    Eigen::Vector3d point = spline(t);
    spline_points.push_back(point);
  }

  return spline_points;
}

int main() {
  // 示例控制点
  std::vector<Eigen::Vector3d> control_points = {
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 2, 3),
      Eigen::Vector3d(4, 5, 6), Eigen::Vector3d(7, 8, 9)};

  // 生成样条曲线上的点
  std::vector<Eigen::Vector3d> spline_points =
      generateSplinePoints(control_points, 100);

  for (const auto &pt : spline_points) {
    std::cout << "Point: " << pt.transpose() << std::endl;
  }

  return 0;
}
