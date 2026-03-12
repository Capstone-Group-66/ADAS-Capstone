cmake_minimum_required(VERSION 3.10)
project(rcw)

set(ncnn_DIR /home/john471g/ncnn/build/install/lib/cmake/ncnn/)

find_package(OpenCV REQUIRED)
find_package(ncnn REQUIRED)

# Copy folders to the build directory automatically
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/videos DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ncnn
    ${OpenCV_LIBS}
)