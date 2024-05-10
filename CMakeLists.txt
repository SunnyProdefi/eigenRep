cmake_minimum_required(VERSION 3.10)
project(MyProject VERSION 1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
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
