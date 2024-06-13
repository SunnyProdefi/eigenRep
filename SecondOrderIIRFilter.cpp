#include <Eigen/Dense>
#include <iostream>

class SecondOrderIIRFilter {
public:
  SecondOrderIIRFilter(const Eigen::Vector3d &b, const Eigen::Vector2d &a)
      : b_(b), a_(a) {
    x_.setZero();
    y_.setZero();
  }

  double apply(double input) {
    // Shift old samples
    x_[2] = x_[1];
    x_[1] = x_[0];
    x_[0] = input;

    // Compute new output
    double output = b_[0] * x_[0] + b_[1] * x_[1] + b_[2] * x_[2] -
                    a_[0] * y_[0] - a_[1] * y_[1]; // Adjusted indices

    // Shift old outputs
    y_[1] = y_[0];  // Shift y[0] to y[1]
    y_[0] = output; // Store new output in y[0]

    return output;
  }

private:
  Eigen::Vector3d b_; // Filter coefficients for input
  Eigen::Vector2d a_; // Filter coefficients for output (feedback)
  Eigen::Vector3d x_; // Input buffer
  Eigen::Vector2d y_; // Output buffer
};

int main() {
  // Coefficients for a second-order low-pass filter example
  Eigen::Vector3d b(0.1, 0.2, 0.1);
  Eigen::Vector2d a(0.0000, -0.05); // a0 = 1 is assumed and not used

  SecondOrderIIRFilter filter(b, a);
  Eigen::VectorXd signal(10);         // Input signal
  Eigen::VectorXd filteredSignal(10); // Filtered signal

  // Example input signal
  signal << 10, 2, 3, -5, 5, 4, 3, 2, 1, 0;

  // Apply filter
  for (int i = 0; i < signal.size(); i++) {
    filteredSignal(i) = filter.apply(signal(i));
    std::cout << "Filtered Signal[" << i << "]: " << filteredSignal(i)
              << std::endl;
  }

  return 0;
}
