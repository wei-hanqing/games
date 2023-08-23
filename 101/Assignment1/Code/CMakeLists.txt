cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED PATHS "D:\\Librarys\\opencv\\build")
find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_STANDARD 17)

include_directories("D:\\Librarys\\opencv\\build\\include";"C:\\Program Files (x86)\\Eigen3\\include")

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer "C:\\bin\\opencv_world470d.lib")
