#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
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
public:
  std::vector<double> s;
  Spline sx, sy;

  std::vector<double> calcS(const std::vector<double> &x,
                            const std::vector<double> &y) {
    std::vector<double> dx(x.size() - 1);
    std::vector<double> dy(y.size() - 1);
    std::transform(x.begin() + 1, x.end(), x.begin(), dx.begin(),
                   std::minus<double>());
    std::transform(y.begin() + 1, y.end(), y.begin(), dy.begin(),
                   std::minus<double>());
    std::vector<double> ds(dx.size());
    for (size_t i = 0; i < dx.size(); ++i) {
      ds[i] = std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
    }
    std::vector<double> s;
    s.push_back(0);
    std::partial_sum(ds.begin(), ds.end(), std::back_inserter(s));
    return s;
  }

  Spline2D(const std::vector<double> &x, const std::vector<double> &y)
      : s(calcS(x, y)), sx(s, x), sy(s, y) {}

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

std::vector<double> linspace(double start, double end, double step) {
  std::vector<double> result;
  for (double val = start; val <= end; val += step) {
    result.push_back(val);
  }
  return result;
}

void calc_spline_course(const std::vector<double> &x,
                        const std::vector<double> &y, double ds,
                        std::vector<double> &rx, std::vector<double> &ry,
                        std::vector<double> &ryaw, std::vector<double> &rk,
                        std::vector<double> &s) {
  std::cout << "calc_spline_course" << std::endl;
  Spline2D sp(x, y); // 创建二维样条曲线对象
  std::cout << "s.back(): " << sp.s.back() << std::endl;
  s = linspace(0, sp.s.back(), ds); //生成从0到样条曲线总长度，间隔为ds的数组

  for (double i_s : s) {                  // 遍历每一个曲线长度值
    auto [ix, iy] = sp.calcPosition(i_s); // 计算当前位置的x, y坐标
    rx.push_back(ix); // 将计算得到的x坐标添加到列表中
    ry.push_back(iy); // 将计算得到的y坐标添加到列表中
    ryaw.push_back(sp.calcYaw(i_s)); // 计算并添加当前位置的偏航角
    rk.push_back(sp.calcCurvature(i_s)); // 计算并添加当前位置的曲率
  }
}

double pi_2_pi(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

std::vector<double> calc_speed_profile(const std::vector<double> &cx,
                                       const std::vector<double> &cy,
                                       const std::vector<double> &cyaw,
                                       double target_speed) {
  std::vector<double> speed_profile(
      cx.size(), target_speed); // 初始化速度剖面，默认值为目标速度
  double direction = 1.0;       // 前进方向

  // 设置停止点
  for (size_t i = 0; i < cx.size() - 1; ++i) {
    double dx = cx[i + 1] - cx[i]; // 计算相邻两个点的x方向差值
    double dy = cy[i + 1] - cy[i]; // 计算相邻两个点的y方向差值

    double move_direction = std::atan2(dy, dx); // 计算运动方向的角度

    if (dx != 0.0 && dy != 0.0) { // 如果dx和dy都不为零
      double dangle = std::abs(pi_2_pi(
          move_direction - cyaw[i])); // 计算运动方向与路径偏航角的角度差
      if (dangle >= M_PI / 4.0) { // 如果角度差大于等于45度
        direction = -1.0;         // 方向设为后退
      } else {
        direction = 1.0; // 否则设为前进
      }
    }

    if (direction != 1.0) {             // 如果方向不是前进
      speed_profile[i] = -target_speed; // 设置当前点的速度为负目标速度
    } else {
      speed_profile[i] = target_speed; // 否则设置为正目标速度
    }
  }

  speed_profile.back() = 0.0; // 最后一个点的速度设为0

  return speed_profile; // 返回速度剖面
}

struct P {
  // 系统配置
  static constexpr int NX = 4; // 状态向量的维数: z = [x, y, v, phi]
  static constexpr int NU = 2; // 输入向量的维数: u = [加速度, 转向角]
  static constexpr int T = 6; // 有限时间域长度

  // MPC配置
  Eigen::Matrix4d Q;  // 状态惩罚矩阵
  Eigen::Matrix4d Qf; // 终端状态惩罚矩阵
  Eigen::Matrix2d R;  // 输入惩罚矩阵
  Eigen::Matrix2d Rd; // 输入变化惩罚矩阵

  double dist_stop;    // 当距离目标小于dist_stop时允许停止
  double speed_stop;   // 当速度小于speed_stop时允许停止
  double time_max;     // 最大仿真时间
  int iter_max;        // 最大迭代次数
  double target_speed; // 目标速度
  int N_IND;           // 搜索索引数量
  double dt;           // 时间步长
  double d_dist;       // 距离步长
  double du_res;       // 停止迭代的阈值

  // 车辆配置
  double RF; // [米] 从车辆后端到前端的距离
  double RB; // [米] 从车辆后端到后面的距离
  double W;  // [米] 车辆宽度
  double WD; // [米] 左右车轮之间的距离
  double WB; // [米] 轴距
  double TR; // [米] 轮胎半径
  double TW; // [米] 轮胎宽度

  double steer_max;        // 最大转向角 [弧度]
  double steer_change_max; // 最大转向速度 [弧度/秒]
  double speed_max;        // 最大速度 [米/秒]
  double speed_min;        // 最小速度 [米/秒]
  double acceleration_max; // 最大加速度 [米/秒^2]

  // 构造函数
  P()
      : Q((Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
           0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
              .finished()),
        Qf((Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
               .finished()),
        R((Eigen::Matrix2d() << 0.01, 0.0, 0.0, 0.1).finished()),
        Rd((Eigen::Matrix2d() << 0.01, 0.0, 0.0, 0.1).finished()), dist_stop(5),
        speed_stop(0.5 / 3.6), time_max(60), iter_max(5),
        target_speed(10.0 / 3.6), N_IND(10), dt(0.2), d_dist(1), du_res(0.1),
        RF(3.3), RB(0.8), W(2.4), WD(0.7 * W), WB(2.5), TR(0.44), TW(0.7),
        steer_max(deg2rad(45.0)), steer_change_max(deg2rad(30.0)),
        speed_max(55.0 / 3.6), speed_min(-20.0 / 3.6), acceleration_max(1.0) {}

private:
  // 辅助函数：将度数转换为弧度
  double deg2rad(double deg) const { return deg * M_PI / 180.0; }
};

P param_global;
class Node {
public:
  double x, y, yaw, v;
  double direct;
  P p;

  Node(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0,
       double direct = 1.0)
      : x(x), y(y), yaw(yaw), v(v), direct(direct) {}

  void update(double a, double delta, double direct) {
    delta = limit_input_delta(delta);
    x += v * cos(yaw) * p.dt;
    y += v * sin(yaw) * p.dt;
    yaw += v / p.WB * tan(delta) * p.dt;
    this->direct = direct;
    v += this->direct * a * p.dt;
    v = limit_speed(v);
  }

  double limit_input_delta(double delta) {
    if (delta >= p.steer_max) {
      return p.steer_max;
    }
    if (delta <= -p.steer_max) {
      return -p.steer_max;
    }
    return delta;
  }

  double limit_speed(double v) {
    if (v >= p.speed_max) {
      return p.speed_max;
    }
    if (v <= p.speed_min) {
      return p.speed_min;
    }
    return v;
  }
};

class PATH {
public:
  std::vector<double> cx, cy, cyaw, ck;
  int length, ind_old;
  P p;

  PATH(const std::vector<double> &cx, const std::vector<double> &cy,
       const std::vector<double> &cyaw, const std::vector<double> &ck)
      : cx(cx), cy(cy), cyaw(cyaw), ck(ck), length(cx.size()), ind_old(0) {}

  std::pair<int, double> nearest_index(const Node &node) {
    int search_up_to = std::min(ind_old + p.N_IND, length);
    std::vector<double> dx, dy, dist;

    for (int i = ind_old; i < search_up_to; ++i) {
      dx.push_back(node.x - cx[i]);
      dy.push_back(node.y - cy[i]);
      dist.push_back(std::hypot(dx.back(), dy.back()));
    }

    auto min_iter = std::min_element(dist.begin(), dist.end());
    int ind_in_N = std::distance(dist.begin(), min_iter);
    int ind = ind_old + ind_in_N;
    ind_old = ind;

    // 计算后轴向量旋转90度的结果
    Eigen::Vector2d rear_axle_vec_rot_90(std::cos(node.yaw + M_PI / 2.0),
                                         std::sin(node.yaw + M_PI / 2.0));

    // 计算目标点到后轴的向量
    Eigen::Vector2d vec_target_2_rear(dx[ind_in_N], dy[ind_in_N]);

    // 计算横向误差
    double er = rear_axle_vec_rot_90.dot(vec_target_2_rear);

    return {ind, er}; // 返回最近点的索引和横向误差
  }
};

std::vector<std::vector<double>>
calc_ref_trajectory_in_T_step(const Node &node, PATH &ref_path, const P &p,
                              const std::vector<double> &sp, int &target_ind) {
  std::vector<std::vector<double>> z_ref(
      p.NX, std::vector<double>(p.T + 1,
                                0.0)); // Initialize reference trajectory array

  std::pair<int, double> nearest_index =
      ref_path.nearest_index(node); // Get the nearest index on the path
                                    // to the current node
  target_ind = nearest_index.first; // Set the target index
  // Initialize reference trajectory
  z_ref[0][0] = ref_path.cx[target_ind];   // x-coordinate
  z_ref[1][0] = ref_path.cy[target_ind];   // y-coordinate
  z_ref[2][0] = sp[target_ind];            // Speed
  z_ref[3][0] = ref_path.cyaw[target_ind]; // Yaw angle

  double dist_move = 0.0; // Initialize moved distance

  for (int i = 1; i <= p.T; ++i) {
    dist_move += std::abs(node.v) * p.dt; // Calculate the moved distance
    int ind_move = static_cast<int>(
        std::round(dist_move / p.d_dist)); // Calculate the index increment
    int index = std::min(target_ind + ind_move,
                         ref_path.length -
                             1); // Ensure the index does not exceed path length

    // Update reference trajectory
    z_ref[0][i] = ref_path.cx[index];   // x-coordinate
    z_ref[1][i] = ref_path.cy[index];   // y-coordinate
    z_ref[2][i] = sp[index];            // Speed
    z_ref[3][i] = ref_path.cyaw[index]; // Yaw angle
  }

  return z_ref; // Return the reference trajectory
}

// Returns A, B matrices for linear discrete-time dynamic model
void calc_linear_discrete_model(double v, double phi, double delta,
                                Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
  // Initialize matrices A and B, and vector C
  A = Eigen::MatrixXd(4, 4);
  B = Eigen::MatrixXd(4, 2);

  // Compute elements of A
  A << 1.0, 0.0, param_global.dt * std::cos(phi),
      -param_global.dt * v * std::sin(phi), 0.0, 1.0,
      param_global.dt * std::sin(phi), param_global.dt * v * std::cos(phi), 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0,
      param_global.dt * std::tan(delta) / param_global.WB, 1.0;

  // Compute elements of B
  B << 0.0, 0.0, 0.0, 0.0, param_global.dt, 0.0, 0.0,
      param_global.dt * v / (param_global.WB * std::pow(std::cos(delta), 2));
}

void linear_mpc_control(const std::vector<std::vector<double>> &x_ref_T,
                        const std::vector<double> &z0,
                        std::vector<double> &a_opt,
                        std::vector<double> &delta_opt, double &a_exc_old,
                        double &delta_exc_old) {
  std::cout << "linear_mpc_control" << std::endl;
  std::cout << "a_exc: " << a_exc_old << ", delta_exc: " << delta_exc_old
            << std::endl;
  // Initialize matrices A, B
  Eigen::MatrixXd A, B;
  //打印z0[2], z0[3], delta_exc_old
  std::cout << "z0[2]: " << z0[2] << ", z0[3]: " << z0[3]
            << ", delta_exc_old: " << delta_exc_old << std::endl;
  calc_linear_discrete_model(z0[2], z0[3], delta_exc_old, A, B);
  // 打印A和B
  std::cout << "A: " << std::endl << A << std::endl;
  std::cout << "B: " << std::endl << B << std::endl;

  // Initialize matrices Q, R, Rd
  Eigen::MatrixXd Q = param_global.Q;
  Eigen::MatrixXd R = param_global.R;
  Eigen::MatrixXd Qf = param_global.Qf;
  //打印Q, R, Qf
  std::cout << "Q: " << std::endl << Q << std::endl;
  std::cout << "R: " << std::endl << R << std::endl;
  std::cout << "Qf: " << std::endl << Qf << std::endl;

  // Initialize vector x0
  Eigen::VectorXd x0(4);
  x0 << z0[0], z0[1], z0[2], z0[3];
  //打印x0
  std::cout << "x0: " << std::endl << x0 << std::endl;

  // Initialize vector x_ref_T
  Eigen::MatrixXd x_ref_T_mat(4, param_global.T + 1);

  // 打印x_ref_T
  for (size_t i = 0; i < x_ref_T.size(); ++i) {
    for (size_t j = 0; j < x_ref_T[i].size(); ++j) {
      std::cout << x_ref_T[i][j] << " ";
      x_ref_T_mat(i, j) = x_ref_T[i][j];
    }
    std::cout << std::endl;
  }
  Eigen::VectorXd x_ref_T_vec = Eigen::VectorXd::Zero((param_global.T + 1) * 4);
  std::cout << "x_ref_T_vec size: " << x_ref_T_vec.size() << std::endl;
  for (int i = 0; i < param_global.T + 1; ++i) {
    x_ref_T_vec.segment(i * 4, 4) = x_ref_T_mat.col(i);
  }
  std::cout << "x_ref_T_vec: " << std::endl << x_ref_T_vec << std::endl;

  // Xt = [x0, x1, x2, ..., xT]
  Eigen::MatrixXd Xt(4, param_global.T + 1);
  // Ut = [u0, u1, u2, ..., uT-1]
  Eigen::MatrixXd Ut(2, param_global.T);

  // x(k+1) = A * x(k) + B * u(k)
  // Xt = M * xt + C * Ut
  // M = [I A A^2 ... A^T]T
  Eigen::MatrixXd M;
  // 填充M
  int state_dim = A.rows(); // Dimension of the state (4 in this case)
  int M_rows = state_dim * (param_global.T + 1);
  int M_cols = state_dim;

  // Resize M just in case it's not the right size
  M.resize(M_rows, M_cols);
  M.setZero();

  // Start with I (the identity matrix) for A^0
  Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(state_dim, state_dim);

  // Populate M with powers of A
  for (int i = 0; i <= param_global.T; ++i) {
    // Set the block starting at (i*state_dim, 0)
    M.block(i * state_dim, 0, state_dim, state_dim) = A_power;
    // Compute the next power of A for the next iteration
    A_power = A_power * A;
  }
  std::cout << "Matrix M:\n" << M << std::endl;
  std::cout << "Matrix M size: " << M.rows() << " x " << M.cols() << std::endl;
  // C = [0 0 0 ... 0; B 0 0 ... 0; AB B 0 ... 0; A^2B AB B ... 0; ...; A^T-1B
  // ... 0 B]
  Eigen::MatrixXd C;
  // 填充C
  int input_dim = B.cols(); // Dimension of the input (2 in this case)
  int C_rows = state_dim * (param_global.T + 1);
  int C_cols = param_global.T * input_dim;
  C.resize(C_rows, C_cols);
  C.setZero();
  std::cout << "Matrix C size: " << C.rows() << " x " << C.cols() << std::endl;

  Eigen::MatrixXd A_power_B = B; // Start with B, which is A^0 * B

  for (int i = param_global.T; i >= 1;
       i--) { // Note the loop starts from 1 because the first row block is
    // zeros
    std::cout << "i: " << i << std::endl;
    for (int j = 0; j < i; ++j) {
      // Place the matrix A^(i-j-1)*B at position (i*state_dim, j*input_dim)
      // Note the (i-j-1) as we start from A^0 = B at j = i-1 and decrease
      C.block((j + 1 + (param_global.T - i)) * state_dim, (j)*input_dim,
              state_dim, input_dim) = A_power_B;
      // std::cout << "Matrix C:\n" << C << std::endl;
    }
    // Update A_power_B for the next row
    A_power_B = A * A_power_B;
    std::cout << "Matrix A_power_B:\n" << A_power_B << std::endl;
  }

  std::cout << "Matrix C:\n" << C << std::endl;
  std::cout << "Matrix C size: " << C.rows() << " x " << C.cols() << std::endl;

  // Initialize matrix Q_bar
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(
      (param_global.T + 1) * Q.rows(), (param_global.T + 1) * Q.cols());
  std::cout << "Matrix Q_bar size: " << Q_bar.rows() << " x " << Q_bar.cols()
            << std::endl;
  for (int i = 0; i <= param_global.T; ++i) {
    if (i == param_global.T) {
      Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Qf;
    }
    Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Q;
  }
  std::cout << "Matrix Q_bar:\n" << Q_bar << std::endl;

  // Initialize matrix R_bar
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(param_global.T * R.rows(),
                                                param_global.T * R.cols());
  std::cout << "Matrix R_bar size: " << R_bar.rows() << " x " << R_bar.cols()
            << std::endl;
  for (int i = 0; i < param_global.T; ++i) {
    R_bar.block(i * R.rows(), i * R.cols(), R.rows(), R.cols()) = R;
  }
  std::cout << "Matrix R_bar:\n" << R_bar << std::endl;

  // Initialize matrix hessian
  Eigen::MatrixXd hessian = C.transpose() * Q_bar * C + R_bar;
  std::cout << "Matrix hessian size: " << hessian.rows() << " x "
            << hessian.cols() << std::endl;
  std::cout << "Matrix hessian:\n" << hessian << std::endl;

  // Initialize matrix gradient
  Eigen::RowVectorXd gradient = (M * x0 - x_ref_T_vec).transpose() * Q_bar * C;
  std::cout << "Matrix gradient size: " << gradient.rows() << " x "
            << gradient.cols() << std::endl;
  std::cout << "Matrix gradient:\n" << gradient << std::endl;

  // Initialize matrix A_ieq
  Eigen::MatrixXd A_ieq = Eigen::MatrixXd::Zero(input_dim * param_global.T,
                                                input_dim * param_global.T);
  std::cout << "Matrix A_ieq size: " << A_ieq.rows() << " x " << A_ieq.cols()
            << std::endl;

  // 填充A_ieq，对角线上的元素为1
  for (int i = 0; i < A_ieq.rows(); ++i) {
    A_ieq(i, i) = 1;
  }
  std::cout << "Matrix A_ieq:\n" << A_ieq << std::endl;

  // Initialize vector lower_bound
  Eigen::VectorXd lower_bound =
      Eigen::VectorXd::Zero(A_ieq.rows()); // Initialize lower bound
  std::cout << "Vector lower_bound size: " << lower_bound.rows() << std::endl;
  // 填充lower_bound
  for (int i = 0; i < lower_bound.rows(); ++i) {
    if (i % 2 == 0) {
      lower_bound(i) = -param_global.acceleration_max;
    } else {
      lower_bound(i) = -param_global.steer_max;
    }
  }
  std::cout << "Vector lower_bound:\n" << lower_bound << std::endl;
  // Initialize vector upper_bound
  Eigen::VectorXd upper_bound =
      Eigen::VectorXd::Zero(A_ieq.rows()); // Initialize upper bound
  std::cout << "Vector upper_bound size: " << upper_bound.rows() << std::endl;
  // 填充upper_bound
  for (int i = 0; i < upper_bound.rows(); ++i) {
    if (i % 2 == 0) {
      upper_bound(i) = param_global.acceleration_max;
    } else {
      upper_bound(i) = param_global.steer_max;
    }
  }
  std::cout << "Vector upper_bound:\n" << upper_bound << std::endl;
  //打印标准QP问题
  std::cout << "Standard QP problem:" << std::endl;
  std::cout << "Hessian:\n" << hessian << std::endl;
  std::cout << "Gradient:\n" << gradient << std::endl;
  std::cout << "A_ieq:\n" << A_ieq << std::endl;
  std::cout << "Lower bound:\n" << lower_bound << std::endl;
  std::cout << "Upper bound:\n" << upper_bound << std::endl;
  std::cout << "Hessian.size(): " << hessian.rows() << " x " << hessian.cols()
            << std::endl;
  std::cout << "Gradient.size(): " << gradient.rows() << " x "
            << gradient.cols() << std::endl;
  std::cout << "A_ieq.size(): " << A_ieq.rows() << " x " << A_ieq.cols()
            << std::endl;
  std::cout << "Lower bound.size(): " << lower_bound.rows() << std::endl;
  std::cout << "Upper bound.size(): " << upper_bound.rows() << std::endl;

  Eigen::SparseMatrix<double> sparse_hessian = hessian.sparseView();
  Eigen::SparseMatrix<double> sparse_A_ieq = A_ieq.sparseView();
  // Solve the QP problem using OSQP
  // Create a new solver
  // 设置OSQP求解器
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(true);

  // 设置QP问题
  solver.data()->setNumberOfVariables(hessian.rows());
  solver.data()->setNumberOfConstraints(hessian.rows());
  if (!solver.data()->setHessianMatrix(sparse_hessian))
    std::cout << "Failed to set Hessian matrix" << std::endl;
  if (!solver.data()->setGradient(gradient))
    std::cout << "Failed to set gradient" << std::endl;
  if (!solver.data()->setLinearConstraintsMatrix(sparse_A_ieq))
    std::cout << "Failed to set linear constraints matrix" << std::endl;
  if (!solver.data()->setLowerBound(lower_bound))
    std::cout << "Failed to set lower bound" << std::endl;
  if (!solver.data()->setUpperBound(upper_bound))
    std::cout << "Failed to set upper bound" << std::endl;

  if (!solver.initSolver())
    std::cout << "Failed to initialize solver" << std::endl;

  // 求解QP问题
  solver.solveProblem(); // Updated to use the new API
  Eigen::VectorXd solution = solver.getSolution();
  std::cout << "Solution:" << solution << std::endl;
  a_opt.push_back(solution(0));
  delta_opt.push_back(solution(1));
}

int main() {
  std::vector<double> ax = {0.0, 15.0, 30.0, 50.0, 60.0}; // 参考路径的x坐标
  std::vector<double> ay = {0.0, 40.0, 15.0, 30.0, 0.0}; // 参考路径的y坐标
  double ds = 1;                                         // 间隔距离

  std::vector<double> cx, cy, cyaw, ck, s;
  std::cout << "calc_spline_course" << std::endl;
  calc_spline_course(ax, ay, ds, cx, cy, cyaw, ck,
                     s); // 计算样条曲线路径

  // 输出结果
  for (size_t i = 0; i < cx.size(); ++i) {
    std::cout << "x: " << cx[i] << ", y: " << cy[i] << ", yaw: " << cyaw[i]
              << ", curvature: " << ck[i] << ", s: " << s[i] << std::endl;
  }
  // 写入.csv文件
  std::ofstream ofs("spline_course.csv");
  ofs << "x, y, yaw, curvature, s" << std::endl;
  for (size_t i = 0; i < cx.size(); ++i) {
    ofs << cx[i] << ", " << cy[i] << ", " << cyaw[i] << ", " << ck[i] << ", "
        << s[i] << std::endl;
  }
  std::vector<double> speed_profile =
      calc_speed_profile(cx, cy, cyaw, 10.0 / 3.6);

  for (size_t i = 0; i < speed_profile.size(); ++i) {
    std::cout << "speed: " << speed_profile[i] << std::endl;
  }

  PATH ref_path(cx, cy, cyaw, ck);
  Node node(cx[0], cy[0], cyaw[0], 0.0);

  std::pair<int, double> nearest_index = ref_path.nearest_index(node);
  std::cout << "nearest_index: " << nearest_index.first
            << ", lateral error: " << nearest_index.second << std::endl;

  double time = 0.0;
  std::vector<double> x = {node.x}, y = {node.y}, yaw = {node.yaw},
                      v = {node.v};
  std::vector<double> t = {0.0}, d = {0.0}, a = {0.0};

  std::vector<double> delta_opt, a_opt;
  double a_exc_old = 0.0, delta_exc_old = 0.0;

  // int target_ind;
  // std::vector<std::vector<double>> ref_pat_T = calc_ref_trajectory_in_T_step(
  //     node, ref_path, param_global, speed_profile, target_ind);
  // // 打印ref_pat_T
  // for (size_t i = 0; i < ref_pat_T.size(); ++i) {
  //   for (size_t j = 0; j < ref_pat_T[i].size(); ++j) {
  //     std::cout << ref_pat_T[i][j] << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::vector<double> z0 = {node.x, node.y, node.v, node.yaw};
  // linear_mpc_control(ref_pat_T, z0, a_opt, delta_opt, a_exc_old,
  // delta_exc_old);
  while (time < param_global.time_max) {
    std::vector<double> z_ref;
    int target_ind;
    std::vector<std::vector<double>> ref_pat_T = calc_ref_trajectory_in_T_step(
        node, ref_path, param_global, speed_profile, target_ind);
    // 打印ref_pat_T
    for (size_t i = 0; i < ref_pat_T.size(); ++i) {
      for (size_t j = 0; j < ref_pat_T[i].size(); ++j) {
        std::cout << ref_pat_T[i][j] << " ";
      }
      std::cout << std::endl;
    }

    std::vector<double> z0 = {node.x, node.y, node.v, node.yaw};
    linear_mpc_control(ref_pat_T, z0, a_opt, delta_opt, a_exc_old,
                       delta_exc_old);

    if (!delta_opt.empty()) {
      delta_exc_old = delta_opt.back();
      a_exc_old = a_opt.back();
      std::cout << "a_exc_old: " << a_exc_old
                << ", delta_exc_old: " << delta_exc_old << std::endl;
    }

    node.update(a_exc_old, delta_exc_old, param_global.dt);
    time += param_global.dt;

    x.push_back(node.x);
    y.push_back(node.y);
    yaw.push_back(node.yaw);
    v.push_back(node.v);
    t.push_back(time);
    d.push_back(delta_exc_old);
    a.push_back(a_exc_old);

    double dist = std::hypot(node.x - cx.back(), node.y - cy.back());
    if (dist < param_global.dist_stop &&
        std::abs(node.v) < param_global.speed_stop) {
      break;
    }
  }
  std::cout << "Vector a_opt:" << std::endl;
  for (auto it = a_opt.begin(); it != a_opt.end(); ++it) {
    std::cout << *it << std::endl;
  }

  std::cout << "Vector delta_opt:" << std::endl;
  for (auto it = delta_opt.begin(); it != delta_opt.end(); ++it) {
    std::cout << *it << std::endl;
  }

  std::ofstream ofs2("result.csv");
  ofs2 << "x, y, yaw, v, t, delta, a" << std::endl;
  for (size_t i = 0; i < x.size(); ++i) {
    ofs2 << x[i] << ", " << y[i] << ", " << yaw[i] << ", " << v[i] << ", "
         << t[i] << ", " << d[i] << ", " << a[i] << std::endl;
  }
  std::cout << "time: " << time << std::endl;

  return 0;
}