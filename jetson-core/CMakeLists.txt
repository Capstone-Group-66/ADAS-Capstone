cmake_minimum_required(VERSION 3.10)
project(jetson_core VERSION 1.0.0 LANGUAGES CXX)

# ══════════════════════════════════════════════════════════════════════════════
#                              COMPILER SETTINGS
# ══════════════════════════════════════════════════════════════════════════════
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization for Jetson
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
endif()

# Enable warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Coverage option
option(COVERAGE "Enable code coverage" OFF)
if(COVERAGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

# ══════════════════════════════════════════════════════════════════════════════
#                              DEPENDENCIES
# ══════════════════════════════════════════════════════════════════════════════
find_package(OpenCV REQUIRED COMPONENTS core videoio imgcodecs highgui calib3d imgproc)
find_package(Threads REQUIRED)

# Optional: yaml-cpp for enhanced config parsing
find_package(yaml-cpp QUIET)

# ══════════════════════════════════════════════════════════════════════════════
#                              INCLUDE DIRECTORIES
# ══════════════════════════════════════════════════════════════════════════════
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})

# ══════════════════════════════════════════════════════════════════════════════
#                              SOURCE FILES
# ══════════════════════════════════════════════════════════════════════════════
set(COMMON_SOURCES
    src/common/Config.cpp
)

set(STAGE_A_SOURCES
    src/stage_a/CameraIngest.cpp
    src/stage_a/CameraCalibrator.cpp
    src/stage_a/DeviceWizard.cpp
    src/stage_a/NetworkIngest.cpp
    src/stage_a/RadarIngest.cpp
    src/stage_a/IMUIngest.cpp
    src/stage_a/IngestManager.cpp
)

# ══════════════════════════════════════════════════════════════════════════════
#                              MAIN EXECUTABLE
# ══════════════════════════════════════════════════════════════════════════════
add_executable(jetson_core
    src/main.cpp
    ${COMMON_SOURCES}
    ${STAGE_A_SOURCES}
)

target_link_libraries(jetson_core
    ${OpenCV_LIBS}
    Threads::Threads
    stdc++fs
)

if(yaml-cpp_FOUND)
    target_link_libraries(jetson_core yaml-cpp)
    target_compile_definitions(jetson_core PRIVATE HAS_YAML_CPP)
endif()

# ══════════════════════════════════════════════════════════════════════════════
#                         DEVICE WIZARD EXECUTABLE
# ══════════════════════════════════════════════════════════════════════════════
add_executable(device_wizard
    src/device_wizard_main.cpp
    src/common/Config.cpp
    src/stage_a/DeviceWizard.cpp
    src/stage_a/CameraCalibrator.cpp
)

target_link_libraries(device_wizard
    ${OpenCV_LIBS}
    Threads::Threads
    stdc++fs
)

# ══════════════════════════════════════════════════════════════════════════════
#                         MAIN BRAIN EXECUTABLE (Vertical Slice)
# ══════════════════════════════════════════════════════════════════════════════
set(MAIN_BRAIN_SOURCES
    src/main_brain/SimpleBleServer.cpp
    src/main_brain/AlertGenerator.cpp
    src/main_brain/BleFragmenter.cpp
)

add_executable(main_brain
    src/main_brain/main_brain_main.cpp
    ${MAIN_BRAIN_SOURCES}
)

# Include vendor directory for nlohmann/json (CBOR encoding)
target_include_directories(main_brain PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor)

target_link_libraries(main_brain
    Threads::Threads
)

# ══════════════════════════════════════════════════════════════════════════════
#                              TESTS (Optional)
# ══════════════════════════════════════════════════════════════════════════════
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()

    # Test SPSC Queue
    add_executable(test_spsc_queue tests/test_spsc_queue.cpp)
    target_link_libraries(test_spsc_queue Threads::Threads)
    add_test(NAME test_spsc_queue COMMAND test_spsc_queue)

    # Test Config
    add_executable(test_config tests/test_config.cpp ${COMMON_SOURCES})
    target_link_libraries(test_config Threads::Threads)
    add_test(NAME test_config COMMAND test_config)
endif()

# ══════════════════════════════════════════════════════════════════════════════
#                              INSTALL
# ══════════════════════════════════════════════════════════════════════════════
install(TARGETS jetson_core device_wizard DESTINATION bin)
install(DIRECTORY config/ DESTINATION etc/adas)

# ══════════════════════════════════════════════════════════════════════════════
#                              STATUS OUTPUT
# ══════════════════════════════════════════════════════════════════════════════
message(STATUS "")
message(STATUS "ADAS Jetson Core Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenCV: ${OpenCV_VERSION}")
message(STATUS "  yaml-cpp: ${yaml-cpp_FOUND}")
message(STATUS "  Coverage: ${COVERAGE}")
message(STATUS "  Tests: ${BUILD_TESTS}")
message(STATUS "")
