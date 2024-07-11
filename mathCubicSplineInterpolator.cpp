#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <math.h>
#include <numeric>
#include <stdexcept>
#include <vector>
class Spline {
private:
  std::vector<double> a, b, c, d, w;
  const std::vector<double> &x; // 使用常量引用
  const std::vector<double> &y; // 使用常量引用
  size_t nx;

  std::vector<double> calcDiff(const std::vector<double> &v) {
    std::vector<double> result(v.size() - 1);
    for (size_t i = 0; i < v.size() - 1; ++i) {
      result[i] = v[i + 1] - v[i];
    }
    return result;
  }

  std::vector<std::vector<double>> calcA(const std::vector<double> &h) {
    std::vector<std::vector<double>> A(nx, std::vector<double>(nx, 0.0));
    A[0][0] = 1.0;
    for (size_t i = 1; i < nx - 1; ++i) {
      A[i][i] = 2.0 * (h[i - 1] + h[i]);
      A[i][i - 1] = h[i - 1];
      A[i - 1][i] = h[i - 1];
    }
    A[nx - 1][nx - 1] = 1.0;
    return A;
  }

  std::vector<double> calcB(const std::vector<double> &h) {
    std::vector<double> B(nx, 0.0);
    for (size_t i = 1; i < nx - 1; ++i) {
      B[i] = 3.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }
    return B;
  }

  void solve(std::vector<std::vector<double>> &A, std::vector<double> &B) {
    Eigen::MatrixXd matA(A.size(), A.size());
    Eigen::VectorXd vecB(B.size());

    for (size_t i = 0; i < A.size(); i++) {
      for (size_t j = 0; j < A.size(); j++) {
        matA(i, j) = A[i][j];
      }
      vecB(i) = B[i];
    }

    Eigen::VectorXd vecC = matA.colPivHouseholderQr().solve(vecB);
    for (size_t i = 0; i < c.size(); i++) {
      c[i] = vecC(i);
    }
  }

  size_t searchIndex(double xi) {
    auto it = std::lower_bound(x.begin(), x.end(), xi);
    return std::distance(x.begin(), it) - 1;
  }

public:
  Spline(const std::vector<double> &x, const std::vector<double> &y)
      : x(x), y(y) {
    nx = x.size();
    std::vector<double> h = calcDiff(x);

    a = y; // Coefficient a directly taken from input y
    std::vector<std::vector<double>> A = calcA(h);
    std::vector<double> B = calcB(h);
    c.resize(nx);
    solve(A, B);

    b.resize(nx - 1);
    d.resize(nx - 1);
    for (size_t i = 0; i < nx - 1; ++i) {
      d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
      b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
    }
  }

  double calc(double t) {
    if (t < x.front() || t > x.back()) {
      return NAN; // Using NAN to indicate out of bounds
    }
    size_t i = searchIndex(t);
    double dx = t - x[i];
    return y[i] + b[i] * dx + c[i] * std::pow(dx, 2) + d[i] * std::pow(dx, 3);
  }

  double calcd(double t) {
    if (t < x.front() || t > x.back()) {
      return NAN; // Using NAN to indicate out of bounds
    }
    size_t i = searchIndex(t);
    double dx = t - x[i];
    return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * std::pow(dx, 2);
  }

  double calcdd(double t) {
    if (t < x.front() || t > x.back()) {
      return NAN; // Using NAN to indicate out of bounds
    }
    size_t i = searchIndex(t);
    double dx = t - x[i];
    return 2.0 * c[i] + 6.0 * d[i] * dx;
  }
};

class Spline2D {
private:
  std::vector<double> s;
  std::vector<double> ds;
  Spline sx, sy;

  std::vector<double> calcS(const std::vector<double> &x,
                            const std::vector<double> &y) {
    std::cout << "x.size():" << x.size() << std::endl;
    std::vector<double> dx(x.size() - 1);
    std::cout << "dx.size():" << dx.size() << std::endl;
    std::vector<double> dy(y.size() - 1);
    std::cout << "dy.size():" << dy.size() << std::endl;
    std::transform(x.begin() + 1, x.end(), x.begin(), dx.begin(),
                   std::minus<double>());
    std::transform(y.begin() + 1, y.end(), y.begin(), dy.begin(),
                   std::minus<double>());
    std::cout << "dx.size():" << dx.size() << std::endl;
    ds.resize(dx.size());
    std::cout << "ds.size():" << ds.size() << std::endl;
    // 确保 dx 和 dy 的内容正确
    for (size_t i = 0; i < dx.size(); ++i) {
      std::cout << "dx[" << i << "]: " << dx[i] << std::endl;
      std::cout << "dy[" << i << "]: " << dy[i] << std::endl;
    }
    for (size_t i = 0; i < dx.size(); ++i) {
      std::cout << "dx[" << i << "]:" << dx[i] << std::endl;
      ds[i] = std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
      std::cout << "ds[" << i << "]:" << ds[i] << std::endl;
    }
    std::cout << "ds.size():" << ds.size() << std::endl;
    std::vector<double> s;
    s.push_back(0);
    std::cout << "s.size():" << s.size() << std::endl;
    std::partial_sum(ds.begin(), ds.end(), std::back_inserter(s));
    std::cout << "s.size():" << s.size() << std::endl;
    return s;
  }

public:
  Spline2D(const std::vector<double> &x, const std::vector<double> &y)
      : s(calcS(x, y)), sx(s, x), sy(s, y) {
    std::cout << "s.size():" << s.size() << std::endl;
  }

  std::pair<double, double> calcPosition(double s_val) {
    double x = sx.calc(s_val);
    double y = sy.calc(s_val);
    return {x, y};
  }

  double calcCurvature(double s_val) {
    double dx = sx.calcd(s_val);
    double ddx = sx.calcdd(s_val);
    double dy = sy.calcd(s_val);
    double ddy = sy.calcdd(s_val);
    double denominator = std::pow(dx * dx + dy * dy, 1.5);
    return denominator == 0 ? std::numeric_limits<double>::quiet_NaN()
                            : (ddy * dx - ddx * dy) / denominator;
  }

  double calcYaw(double s_val) {
    double dx = sx.calcd(s_val);
    double dy = sy.calcd(s_val);
    return std::atan2(dy, dx);
  }
};

int main() {
  std::vector<double> x = {0.0, 15.0, 30.0, 50.0, 60.0};
  std::vector<double> y = {0.0, 40.0, 15.0, 30.0, 10.0};
  Spline spline(x, y);
  std::cout << spline.calc(1) << std::endl;
  std::cout << spline.calcd(1) << std::endl;
  std::cout << spline.calcdd(1) << std::endl;

  Spline2D spline2d(x, y);
  //   std::cout << spline2d.calcCurvature(0.5) << std::endl;
  //   std::cout << spline2d.calcYaw(0.5) << std::endl;

  return 0;
}