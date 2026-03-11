cmake_minimum_required(VERSION 3.10)
project(jetson_core VERSION 1.0.0 LANGUAGES CXX)

# ══════════════════════════════════════════════════════════════════════════════
#                              COMPILER SETTINGS
# ══════════════════════════════════════════════════════════════════════════════
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization for Jetson (GCC/Clang only)
if(NOT MSVC)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
    endif()
    # Enable warnings (GCC/Clang)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
    # Coverage option
    option(COVERAGE "Enable code coverage" OFF)
    if(COVERAGE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    endif()
else()
    # MSVC Warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
endif()

# ══════════════════════════════════════════════════════════════════════════════
#                              DEPENDENCIES
# ══════════════════════════════════════════════════════════════════════════════
find_package(OpenCV REQUIRED COMPONENTS core videoio imgcodecs highgui calib3d imgproc dnn video)
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
    src/common/Globals.cpp
    src/common/MetricsLogger.cpp
)

set(STAGE_A_SOURCES
    src/stage_a/CameraIngest.cpp
    src/stage_a/CameraCalibrator.cpp
    src/stage_a/DeviceWizard.cpp
    src/stage_a/NetworkIngest.cpp
    src/stage_a/NetworkReceiver.cpp
    src/stage_a/DeepStreamReceiver.cpp
    src/stage_a/BSDReceiver.cpp
    src/stage_a/RadarIngest.cpp
    src/stage_a/IngestManager.cpp
)

set(STAGE_B_SOURCES
    src/stage_b/CameraPreprocessor.cpp
    src/stage_b/ObjectDetector.cpp
    src/stage_b/ObjectTracker.cpp
    src/stage_b/CameraPipeline.cpp
    # Front camera detections are received through DeepStream IPC.
)

set(STAGE_E_SOURCES
    src/stage_e/SensorFusion.cpp
    src/stage_e/FCWMonitor.cpp
    src/stage_e/Track.cpp
    src/stage_e/EgoFrame.cpp
    src/stage_e/BEVDashboard.cpp
)

# Main brain sources needed by jetson_core for BLE
set(MAIN_BRAIN_SOURCES
    src/main_brain/SimpleBleServer.cpp
    src/main_brain/AlertGenerator.cpp
    src/main_brain/BleFragmenter.cpp
)

set(RECORDING_SOURCES
    src/recording/Recorder.cpp
    src/recording/ReplayEngine.cpp
)

add_executable(jetson_core
    src/main.cpp
    ${COMMON_SOURCES}
    ${STAGE_A_SOURCES}
    ${STAGE_B_SOURCES}
    ${STAGE_E_SOURCES}
    ${MAIN_BRAIN_SOURCES}
    ${RECORDING_SOURCES}
)

# Include vendor directory for nlohmann/json (CBOR encoding)
target_include_directories(jetson_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor)

target_link_libraries(jetson_core
    ${OpenCV_LIBS}
    Threads::Threads
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:stdc++fs>
)

# TensorRT support (Jetson only)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    find_library(NVINFER_LIB nvinfer HINTS /usr/lib/aarch64-linux-gnu)
    find_library(CUDA_RT_LIB cudart HINTS /usr/local/cuda/lib64)
    
    if(NVINFER_LIB AND CUDA_RT_LIB)
        message(STATUS "TensorRT found: ${NVINFER_LIB}")
        target_link_libraries(jetson_core ${NVINFER_LIB} ${CUDA_RT_LIB})
        target_include_directories(jetson_core PRIVATE 
            /usr/include/aarch64-linux-gnu
            /usr/local/cuda/include
        )
    else()
        message(WARNING "TensorRT not found - ObjectDetector will be disabled")
    endif()

    # DeepStream rendering/running is external; no extra C++ linkage needed here.
endif()

if(MSVC)
    target_compile_definitions(jetson_core PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

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
    src/common/Globals.cpp
    src/stage_a/DeviceWizard.cpp
    src/stage_a/CameraCalibrator.cpp
    ${RECORDING_SOURCES}
)

target_link_libraries(device_wizard
    ${OpenCV_LIBS}
    Threads::Threads
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:stdc++fs>
)

if(MSVC)
    target_compile_definitions(device_wizard PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# ══════════════════════════════════════════════════════════════════════════════
#                         MAIN BRAIN SOURCES (BLE + Alert pipeline)
# ══════════════════════════════════════════════════════════════════════════════
set(MAIN_BRAIN_SOURCES
    src/main_brain/SimpleBleServer.cpp
    src/main_brain/AlertGenerator.cpp
    src/main_brain/BleFragmenter.cpp
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

    # Test Radar Ingest
    add_executable(test_radar_ingest tests/test_radar_ingest.cpp src/stage_a/RadarIngest.cpp ${RECORDING_SOURCES} ${COMMON_SOURCES})
    target_include_directories(test_radar_ingest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor)
    target_link_libraries(test_radar_ingest Threads::Threads)
    if(yaml-cpp_FOUND)
        target_link_libraries(test_radar_ingest yaml-cpp)
    endif()
    add_test(NAME test_radar_ingest COMMAND test_radar_ingest)

    # Test BEV dashboard math/lifecycle helpers
    add_executable(test_bev_dashboard tests/test_bev_dashboard.cpp)
    target_link_libraries(test_bev_dashboard Threads::Threads)
    add_test(NAME test_bev_dashboard COMMAND test_bev_dashboard)

    # Test asynchronous SensorFusion EKF + aggressive association
    add_executable(test_sensor_fusion_async
        tests/test_sensor_fusion_async.cpp
        src/stage_e/SensorFusion.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(test_sensor_fusion_async Threads::Threads ${OpenCV_LIBS})
    add_test(NAME test_sensor_fusion_async COMMAND test_sensor_fusion_async)

    # Test FCW v3 camera-hold suppression behavior
    add_executable(test_fcw_monitor_v3
        tests/test_fcw_monitor_v3.cpp
        src/stage_e/FCWMonitor.cpp
        ${COMMON_SOURCES}
    )
    target_link_libraries(test_fcw_monitor_v3 Threads::Threads)
    add_test(NAME test_fcw_monitor_v3 COMMAND test_fcw_monitor_v3)
endif()

# ══════════════════════════════════════════════════════════════════════════════
#                         PI4 NETWORK RECEIVER (ZMQ)
# ══════════════════════════════════════════════════════════════════════════════
# Find ZeroMQ - required for Pi4 network integration
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ QUIET libzmq)
endif()

# Fallback: try find_library
if(NOT ZMQ_FOUND)
    find_library(ZMQ_LIBRARY NAMES zmq libzmq PATHS /usr/lib /usr/local/lib)
    find_path(ZMQ_INCLUDE_DIR zmq.h PATHS /usr/include /usr/local/include)
    if(ZMQ_LIBRARY AND ZMQ_INCLUDE_DIR)
        set(ZMQ_FOUND TRUE)
        set(ZMQ_LIBRARIES ${ZMQ_LIBRARY})
        set(ZMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR})
    endif()
endif()

if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found: ${ZMQ_LIBRARIES}")
    
    # Standalone receiver executable (for debugging)
    add_executable(jetson_receiver
        scripts/jetson_receiver.cpp
    )
    target_include_directories(jetson_receiver PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ZMQ_INCLUDE_DIRS}
    )
    target_link_libraries(jetson_receiver
        ${ZMQ_LIBRARIES}
        Threads::Threads
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:stdc++fs>
    )
    
    # Network receiver integrated with pipeline
    add_library(network_receiver STATIC
        src/stage_a/NetworkReceiver.cpp
        src/stage_a/DeepStreamReceiver.cpp
        ${RECORDING_SOURCES}
    )
    target_include_directories(network_receiver PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ZMQ_INCLUDE_DIRS}
    )
    target_link_libraries(network_receiver
        ${ZMQ_LIBRARIES}
        ${OpenCV_LIBS}
        Threads::Threads
    )
    target_compile_definitions(network_receiver PUBLIC HAS_ZMQ)
    
    # Link network_receiver to jetson_core
    target_link_libraries(jetson_core network_receiver)
    target_compile_definitions(jetson_core PRIVATE HAS_ZMQ)
    
    # Link network_receiver to device_wizard for discovery
    target_link_libraries(device_wizard network_receiver)
    target_compile_definitions(device_wizard PRIVATE HAS_ZMQ)
    
    install(TARGETS jetson_receiver DESTINATION bin)
else()
    message(STATUS "ZeroMQ NOT found - Pi4 network integration disabled")
    message(STATUS "  Install with: sudo apt install libzmq3-dev")
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
