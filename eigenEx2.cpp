#include <iostream>  
#include <Eigen/Dense>  

using Eigen::MatrixXd;  
using Eigen::VectorXd;

int main()  
{
  MatrixXd m = MatrixXd::Random(3,3);  // 创建一个 3x3 的动态大小矩阵，元素为随机值。
  m = (m + MatrixXd::Constant(3,3,1.2)) * 50;  // 将矩阵每个元素增加 1.2 后乘以 50。
  std::cout << "m =" << std::endl << m << std::endl;  // 输出矩阵 m 到控制台。
  VectorXd v(3);  // 创建一个大小为 3 的动态向量。
  v << 1, 2, 3;  // 将向量的元素设置为 1, 2, 3。
  std::cout << "m * v =" << std::endl << m * v << std::endl;  // 输出矩阵 m 与向量 v 的乘积结果到控制台。
}
