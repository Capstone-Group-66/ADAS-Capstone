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
    src/stage_a/RadarIngest.cpp
    src/stage_a/IngestManager.cpp
)

set(STAGE_B_SOURCES
    src/stage_b/CameraPreprocessor.cpp
    src/stage_b/ObjectDetector.cpp
    src/stage_b/ObjectTracker.cpp
    src/stage_b/CameraPipeline.cpp
    src/stage_b/FrontCamDeepStream.cpp   # DeepStream front-camera pipeline
)

set(STAGE_E_SOURCES
    src/stage_e/SensorFusion.cpp
    src/stage_e/FCWMonitor.cpp
    src/stage_e/Track.cpp
    src/stage_e/EgoFrame.cpp
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

    # ── DeepStream 6.0 + GStreamer (Front Camera inference pipeline) ─────────
    # DeepStream ships with its own GStreamer build on Jetson via JetPack.
    # Standard install path: /opt/nvidia/deepstream/deepstream/
    set(DS_ROOT "/opt/nvidia/deepstream/deepstream" CACHE PATH
        "DeepStream SDK root directory")

    find_library(NVDSGST_META_LIB nvdsgst_meta
        HINTS ${DS_ROOT}/lib /opt/nvidia/deepstream/deepstream-6.0/lib)
    find_library(NVDS_META_LIB   nvds_meta
        HINTS ${DS_ROOT}/lib /opt/nvidia/deepstream/deepstream-6.0/lib)

    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(GST QUIET
            gstreamer-1.0
            gstreamer-video-1.0
        )
    endif()

    if(NVDSGST_META_LIB AND NVDS_META_LIB AND GST_FOUND)
        message(STATUS "DeepStream found: ${DS_ROOT}")
        message(STATUS "GStreamer: ${GST_LIBRARIES}")

        target_include_directories(jetson_core PRIVATE
            ${DS_ROOT}/sources/includes    # nvdsmeta.h, gstnvdsmeta.h, etc.
            ${GST_INCLUDE_DIRS}
        )
        target_link_libraries(jetson_core
            ${NVDSGST_META_LIB}
            ${NVDS_META_LIB}
            ${GST_LIBRARIES}
        )
        target_compile_definitions(jetson_core PRIVATE HAS_DEEPSTREAM)
        message(STATUS "DeepStream + GStreamer linked — FrontCamDeepStream ENABLED")
    else()
        message(WARNING
            "DeepStream or GStreamer not found — FrontCamDeepStream will be a stub.\n"
            "  Install DeepStream 6.0 via JetPack or:\n"
            "    sudo apt install deepstream-6.0\n"
            "  GStreamer status: GST_FOUND=${GST_FOUND}\n"
            "  nvdsgst_meta:    ${NVDSGST_META_LIB}\n"
            "  nvds_meta:       ${NVDS_META_LIB}"
        )
    endif()
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
