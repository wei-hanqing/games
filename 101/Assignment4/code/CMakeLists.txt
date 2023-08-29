cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

find_package(OpenCV REQUIRED PATHS "D:\\Librarys\\opencv\\build")

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_LIBRARIES "C:\\bin\\opencv_world470d.lib")
include_directories("D:\\Librarys\\opencv\\build\\include"; "C:\\Program Files (x86)\\Eigen3\\include")
add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})
