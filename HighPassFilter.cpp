#include <Eigen/Dense>
#include <iostream>

class HighPassFilter {
private:
  double alpha;
  double beta;
  double x_prev; // 上一次的输入，用于递归计算
  double y_prev; // 上一次的输出，同样用于递归计算

public:
  // 构造函数，初始化滤波系数和初始值
  HighPassFilter(double alpha, double beta, double initial_input = 0.0,
                 double initial_output = 0.0)
      : alpha(alpha), beta(beta), x_prev(initial_input),
        y_prev(initial_output) {}

  // 应用高通滤波器到输入值
  double filter(double input) {
    double output = alpha * (input - x_prev) + beta * y_prev;
    x_prev = input;  // 更新上一次的输入
    y_prev = output; // 更新上一次的输出
    return output;
  }

  // 重置滤波器状态
  void reset(double initial_input = 0.0, double initial_output = 0.0) {
    x_prev = initial_input;
    y_prev = initial_output;
  }
};

int main() {
  // 创建滤波器实例，假设alpha=0.1，beta=0.9
  HighPassFilter filter(0.1, 0.9);

  // 模拟一系列输入数据并应用滤波器
  Eigen::VectorXd inputs =
      Eigen::VectorXd::LinSpaced(10, 1, 100); // 生成1到100的线性间隔向量
  Eigen::VectorXd outputs(100);

  for (int i = 0; i < inputs.size(); ++i) {
    outputs[i] = filter.filter(inputs[i]);
    std::cout << "Input: " << inputs[i] << ", Filtered Output: " << outputs[i]
              << std::endl;
  }

  return 0;
}
