cmake_minimum_required(VERSION 3.31)

project(SensorFusion)


find_package(OpenCV REQUIRED)

add_executable(SensorFusion main.cpp)
target_link_libraries(SensorFusion ${OpenCV_LIBS})
