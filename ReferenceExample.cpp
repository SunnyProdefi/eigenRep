#include <iostream>

void modify(int &ref) {
  ref = 10; // 修改引用，原始值也会被改变
}

int main() {
  int original = 5;
  std::cout << "Original value: " << original << std::endl; // 输出原始值 5

  modify(original); // 传递变量到函数中，通过引用修改

  std::cout << "Modified value: " << original
            << std::endl; // 输出被修改后的值 10
  return 0;
}
