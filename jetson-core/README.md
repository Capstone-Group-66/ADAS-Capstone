# Jetson Core - ADAS Pipeline

## Primary Reference

For the full engineering reference for `jetson-core`, including the exact Stage E fusion math, FCW risk model, BEV dashboard color legend, runtime architecture, BLE transport, replay format, and support tooling, see [docs/JETSON_CORE_ENGINEERING_REFERENCE.md](docs/JETSON_CORE_ENGINEERING_REFERENCE.md).

## Overview

The Jetson-based processing and sensor pipeline for the ADAS project.
Authoritative deployment target: NVIDIA Jetson Nano running Ubuntu 18.

## Stage A: Ingest and Timestamp

Stage A is responsible for:
- Capturing raw sensor data from all devices
- Applying authoritative timestamps (`t_ingest_ns`) using `CLOCK_MONOTONIC_RAW`
- Pushing structured payloads into lock-free SPSC queues

### Sensors Handled

| Sensor | Mount | Connection | Module |
|--------|-------|------------|--------|
| Front Camera | FrontCam | External DeepStream | DeepStreamReceiver |
| Side Camera L | SideCamL | USB | CameraIngest |
| Side Camera R | SideCamR | USB | CameraIngest |
| Rear Camera | RearCam | Pi4 network | NetworkReceiver / NetworkIngest |
| Front Radar (OPS243-A) | FrontRadar | Serial | RadarIngest |
| Rear Radar L (C4001) | RearCornerRadarL | Pi4 network | NetworkReceiver / BSDReceiver |
| Rear Radar R (C4001) | RearCornerRadarR | Pi4 network | NetworkReceiver / BSDReceiver |
| IMU | IMU | Pi4 or local ingest path | NetworkReceiver / IMU queue |

## Build

Build and run this project on the Jetson Nano target environment. Do not treat this workstation as the authoritative build or runtime host.

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

### Build Options

- `BUILD_TESTS=ON` - Build unit tests
- `COVERAGE=ON` - Enable code coverage

## Run

### 1. Device Registration (first time)

```bash
./device_wizard
```

This maps USB cameras to mount positions and saves `hardware_map.json`.

### 2. Start Pipeline

```bash
./jetson_core
```

Options:
- `--config <path>` - Path to `componentConfig.yaml`
- `--hardware-map <path>` - Path to `hardware_map.json`
- `--calib-dir <path>` - Path to calibration directory
- `--record <dir>` - Record a `.adasrec` session
- `--replay <file>` - Replay a `.adasrec` file
- `--replay-speed <float>` - Replay speed multiplier
- `--replay-fast` - Replay as fast as possible
- `--auto-start` - Start pipeline automatically

Front-camera replay now replays recorded DeepStream detection batches rather
than raw front camera frames. This keeps replay aligned with the live
production boundary, where external DeepStream owns the raw front video path.

## Directory Structure

```text
jetson-core/
|- CMakeLists.txt
|- config/
|- docs/
|- include/adas/
|- src/
|- models/
|- scripts/
|- tests/
|- CUCAD_sign_model/
`- vendor/
```

## Requirements

- C++17
- OpenCV 4.x
- CMake 3.10+
- Ubuntu 18 on the Jetson Nano target environment
- NVIDIA Jetson Nano for the authoritative live deployment path

## Tested On

- NVIDIA Jetson Nano
- Ubuntu 18
