cmake_minimum_required(VERSION 3.10)
project(jetson_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For now, just build a simple executable from main.cpp
add_executable(jetson_core
    src/main.cpp
)
