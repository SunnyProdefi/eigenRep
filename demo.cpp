#include <iostream>
#include <vector>

class MyClass {
private:
  int s;
  std::vector<double> ds = {1.0, 2.0, 3.0};

  int privateFunction(const int &x) {
    ds.resize(x); // 确保ds有足够的空间
    std::cout << "privateFunction called, x = " << x << std::endl;
    std::cout << "ds.size() = " << ds.size() << std::endl;
    if (x > 41) {
      std::cout << "ds[0] = " << ds[0] << std::endl;
      ds[41] = x; // 修改此处为赋值操作
      std::cout << "ds[41] = " << ds[41] << std::endl;
    } else {
      std::cerr << "Error: x is too small to access ds[41]" << std::endl;
    }
    return x;
  }

public:
  MyClass(const int &x) : s(privateFunction(x)) {}

  void setPrivateVar(int value) {
    s = value; // 公有成员函数也可以赋值给私有成员变量
  }

  void callPrivateFunction() {
    privateFunction(10); // 公有成员函数可以调用私有成员函数
  }

  void printPrivateVar() { std::cout << "privateVar: " << s << std::endl; }
};

int main() {
  MyClass obj(42);
  obj.setPrivateVar(10);
  obj.printPrivateVar(); // 输出: privateVar: 10

  obj.callPrivateFunction();
  obj.printPrivateVar(); // 输出: privateVar: 42

  return 0;
}
