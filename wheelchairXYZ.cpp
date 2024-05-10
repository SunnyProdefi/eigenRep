#include <Eigen/Dense>
#include <cmath>
#include <iostream>

double radians(double degrees) { return degrees * M_PI / 180.0; }

void calculatePositions(double a_x, double a_y, double theta_deg,
                        double phi_deg, Eigen::Vector4d &point_a,
                        Eigen::Vector3d &point_b, Eigen::Vector3d &point_c,
                        Eigen::Vector3d &point_d) {
  double theta = radians(theta_deg);
  double phi = radians(phi_deg);
  double d1 = 0.5;
  double d2 = 0.2;

  // Configure point a, with theta stored as the fourth component
  point_a = Eigen::Vector4d(a_x, a_y, 0.568, theta);

  // Calculate point b
  double b_x = a_x + d1 * cos(theta);
  double b_y = a_y + d1 * sin(theta);
  double b_z = 0.6;
  point_b = Eigen::Vector3d(b_x, b_y, b_z);

  // Calculate point c
  double c_x = b_x - d2 * sin(theta + phi);
  double c_y = b_y + d2 * cos(theta + phi);
  double c_z = b_z;
  point_c = Eigen::Vector3d(c_x, c_y, c_z);

  // Calculate point d
  double d_x = b_x + d2 * sin(theta + phi);
  double d_y = b_y - d2 * cos(theta + phi);
  double d_z = b_z;
  point_d = Eigen::Vector3d(d_x, d_y, d_z);
}

void calculatePositions_A(double phi_deg, Eigen::Vector3d &point_b_A,
                          Eigen::Vector3d &point_c_A,
                          Eigen::Vector3d &point_d_A) {
  double phi = radians(phi_deg);
  double d1 = 0.5;
  double d2 = 0.2;
  // Calculate point b
  double b_x = d1;
  double b_y = 0;
  double b_z = 0.032;
  point_b_A = Eigen::Vector3d(b_x, b_y, b_z);

  // Calculate point c
  double c_x = b_x - d2 * sin(phi);
  double c_y = b_y + d2 * cos(phi);
  double c_z = b_z;
  point_c_A = Eigen::Vector3d(c_x, c_y, c_z);

  // Calculate point d
  double d_x = b_x + d2 * sin(phi);
  double d_y = b_y - d2 * cos(phi);
  double d_z = b_z;
  point_d_A = Eigen::Vector3d(d_x, d_y, d_z);
}

int main() {
  Eigen::Vector4d point_a;
  Eigen::Vector3d point_b, point_c, point_d;

  Eigen::Vector3d point_b_A, point_c_A, point_d_A;
  // Set initial values for a
  double a_x = 10.0;       // X coordinate of point a
  double a_y = 20.0;       // Y coordinate of point a
  double theta_deg = 45.0; // Angle theta in degrees
  double phi_deg = 30.0;   // Angle phi in degrees

  calculatePositions(a_x, a_y, theta_deg, phi_deg, point_a, point_b, point_c,
                     point_d);
  // Print points
  std::cout << "Point a: (" << point_a[0] << ", " << point_a[1] << ", "
            << point_a[2] << ", Theta: " << point_a[3] << ")" << std::endl;
  std::cout << "Point b: (" << point_b[0] << ", " << point_b[1] << ", "
            << point_b[2] << ")" << std::endl;
  std::cout << "Point c: (" << point_c[0] << ", " << point_c[1] << ", "
            << point_c[2] << ")" << std::endl;
  std::cout << "Point d: (" << point_d[0] << ", " << point_d[1] << ", "
            << point_d[2] << ")" << std::endl;

  calculatePositions_A(phi_deg, point_b_A, point_c_A, point_d_A);
  // Print points
  std::cout << "Point b_A: (" << point_b_A[0] << ", " << point_b_A[1] << ", "
            << point_b_A[2] << ")" << std::endl;
  std::cout << "Point c_A: (" << point_c_A[0] << ", " << point_c_A[1] << ", "
            << point_c_A[2] << ")" << std::endl;
  std::cout << "Point d_A: (" << point_d_A[0] << ", " << point_d_A[1] << ", "
            << point_d_A[2] << ")" << std::endl;
  return 0;
}
