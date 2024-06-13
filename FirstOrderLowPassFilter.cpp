#include <Eigen/Dense>
#include <iostream>

// 一阶低通滤波器类
class LowPassFilter {
public:
  // 构造函数
  LowPassFilter(double alpha) : alpha(alpha), y_prev(0.0) {}

  // 应用滤波器的函数
  double filter(double x) {
    double y = alpha * x + (1 - alpha) * y_prev;
    y_prev = y; // 更新上一次输出
    return y;
  }

private:
  double alpha;  // 滤波系数
  double y_prev; // 上一次的输出
};

int main() {
  Eigen::VectorXd signal(10);          // 测试信号
  Eigen::VectorXd filtered_signal(10); // 滤波后的信号

  // 创建一个测试信号
  signal << 1, 2, 3, 4, 5, 4, 3, 2, 1, 0;

  // 创建滤波器实例，alpha设为0.5
  LowPassFilter lpf(0.5);

  // 应用滤波器
  for (int i = 0; i < signal.size(); ++i) {
    filtered_signal(i) = lpf.filter(signal(i));
  }

  // 输出原信号和滤波后的信号
  std::cout << "Original Signal: " << signal.transpose() << std::endl;
  std::cout << "Filtered Signal: " << filtered_signal.transpose() << std::endl;

  return 0;
}
