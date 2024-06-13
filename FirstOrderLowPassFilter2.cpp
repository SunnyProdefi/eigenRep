#include <Eigen/Dense>
#include <iostream>

class FirstOrderLowPassFilter {
public:
  // 构造函数
  FirstOrderLowPassFilter(double sampleRate, double cutoffFreq) {
    double RC = 1.0 / (2 * M_PI * cutoffFreq);
    double dt = 1.0 / sampleRate;
    alpha = dt / (RC + dt);
    printf("alpha: %f\n", alpha);
  }

  // 应用滤波器方法
  double apply(double input) {
    previousOutput = alpha * input + (1 - alpha) * previousOutput;
    return previousOutput;
  }

private:
  double alpha;              // 滤波系数
  double previousOutput = 0; // 上一次的输出，初始为0
};

int main() {
  // 示例：采样率为1000Hz，截止频率为5Hz
  FirstOrderLowPassFilter lpf(1000, 5);

  // 模拟输入信号，这里可以使用Eigen的向量来处理数据
  Eigen::VectorXd signal(10), filteredSignal(10);
  signal << 10, 10, 10, 10, 10, 0, 0, 0, 0, 0;

  // 应用滤波器
  for (int i = 0; i < signal.size(); ++i) {
    filteredSignal(i) = lpf.apply(signal(i));
    std::cout << "Filtered signal[" << i << "]: " << filteredSignal(i)
              << std::endl;
  }

  return 0;
}
