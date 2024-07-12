cmake_minimum_required(VERSION 3.29)
project(MyProject VERSION 1.0 LANGUAGES CXX)
# 设置 C++ 标准为 C++11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 确保编译器支持 C++11

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(eigenEx1 eigenEx1.cpp)
target_link_libraries(eigenEx1 Eigen3::Eigen)

add_executable(eigenEx2 eigenEx2.cpp)
target_link_libraries(eigenEx2 Eigen3::Eigen)

add_executable(eigenEx3 eigenEx3.cpp)
target_link_libraries(eigenEx3 Eigen3::Eigen)

add_executable(eigenEx4 eigenEx4.cpp)
target_link_libraries(eigenEx4 Eigen3::Eigen)

add_executable(wheelchairXYZ wheelchairXYZ.cpp)
target_link_libraries(wheelchairXYZ Eigen3::Eigen)

add_executable(minimize_joint_velocities minimize_joint_velocities.cpp)
target_link_libraries(minimize_joint_velocities Eigen3::Eigen)

add_executable(moore_penrose_example moore_penrose_example.cpp)
target_link_libraries(moore_penrose_example Eigen3::Eigen)

add_executable(FirstOrderLowPassFilter FirstOrderLowPassFilter.cpp)
target_link_libraries(FirstOrderLowPassFilter Eigen3::Eigen)

add_executable(FirstOrderLowPassFilter2 FirstOrderLowPassFilter2.cpp)
target_link_libraries(FirstOrderLowPassFilter2 Eigen3::Eigen)

add_executable(ReferenceExample ReferenceExample.cpp)
target_link_libraries(ReferenceExample Eigen3::Eigen)

add_executable(HighPassFilter HighPassFilter.cpp)
target_link_libraries(HighPassFilter Eigen3::Eigen)

add_executable(SecondOrderIIRFilter SecondOrderIIRFilter.cpp)
target_link_libraries(SecondOrderIIRFilter Eigen3::Eigen)

add_executable(CubicSplineInterpolator CubicSplineInterpolator.cpp)
target_link_libraries(CubicSplineInterpolator Eigen3::Eigen)

add_executable(BSplineTrajectoryGenerator BSplineTrajectoryGenerator.cpp)
target_link_libraries(BSplineTrajectoryGenerator Eigen3::Eigen)

add_executable(InteriorPointQP InteriorPointQP.cpp)
target_link_libraries(InteriorPointQP Eigen3::Eigen)

add_executable(extract_rotation_axis_angle extract_rotation_axis_angle.cpp)
target_link_libraries(extract_rotation_axis_angle Eigen3::Eigen)

add_executable(ExtractRotationAxisAndAngle ExtractRotationAxisAndAngle.cpp)
target_link_libraries(ExtractRotationAxisAndAngle Eigen3::Eigen)

add_executable(wbc wbc.cpp)
target_link_libraries(wbc Eigen3::Eigen)

add_executable(mathCubicSplineInterpolator mathCubicSplineInterpolator.cpp)
target_link_libraries(mathCubicSplineInterpolator Eigen3::Eigen OsqpEigen::OsqpEigen)

add_executable(OSQP OSQP.cpp)
target_link_libraries(OSQP Eigen3::Eigen OsqpEigen::OsqpEigen)

add_executable(demo demo.cpp)
target_link_libraries(demo Eigen3::Eigen)