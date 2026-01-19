# Pi4 ↔ Jetson Integration Specification

> **For:** Pi Team  
> **From:** Jetson Team  
> **Version:** 1.1  
> **Date:** 2026-01-18  
> **Shared Header:** `include/adas/common/PiProtocol.hpp`

---

## 1. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         JETSON ORIN (Brain)                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │  FrontCam   │  │  SideCamL   │  │  SideCamR   │  (USB Direct)   │
│  │  FrontRadar │  │             │  │             │                 │
│  └─────────────┘  └─────────────┘  └─────────────┘                 │
│                           │                                         │
│                    ┌──────▼──────┐                                  │
│                    │   Stage A   │ ← Ingest & Timestamp             │
│                    │  (Pipeline) │   (Pi data pre-timestamped)      │
│                    └──────▲──────┘                                  │
│                           │                                         │
│              ZMQ PULL (tcp://0.0.0.0:5555-5559)                    │
└───────────────────────────┼─────────────────────────────────────────┘
                            │ Network (WiFi/Ethernet)
                            │
┌───────────────────────────▼─────────────────────────────────────────┐
│                         PI4 (Rear Hub)                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌───────────┐  │
│  │   RearCam   │  │ RearRadarL  │  │ RearRadarR  │  │    IMU    │  │
│  │  (USB/CSI)  │  │ (ttyUSB0)   │  │ (ttyUSB1)   │  │ (BNO085)  │  │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬─────┘  │
│         │                │                │               │         │
│         ▼                ▼                ▼               ▼         │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                  Pi Sensor Publisher                        │   │
│  │  - Timestamps with Chrony-synced clock (AUTHORITATIVE)     │   │
│  │  - Serializes to wire format (see PiProtocol.hpp)          │   │
│  │  - Publishes via ZMQ PUSH                                   │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  Chrony CLIENT → synced to Jetson (offset < 5ms)                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Pi-Hosted Devices (Hardcoded)

The Pi **always** hosts these devices:
- **RearCam** - Rear-facing camera
- **RearRadarL** - Left rear corner radar
- **RearRadarR** - Right rear corner radar  
- **IMU** - BNO085 inertial measurement unit

---

## 2. Shared Header File

**CRITICAL:** Copy this file to your Pi codebase to ensure struct compatibility:

```
jetson-core/include/adas/common/PiProtocol.hpp
```

This header contains:
- All struct definitions with `#pragma pack`
- Message type enums
- Port constants
- Validation helpers

**Both teams must use identical struct definitions!**

---

## 3. Network & Port Allocation

| Port | Direction | Protocol | Content |
|------|-----------|----------|---------|
| **5555** | Pi → Jetson | ZMQ PUSH/PULL | RearCam frames |
| **5556** | Pi → Jetson | ZMQ PUSH/PULL | RearRadarL data |
| **5557** | Pi → Jetson | ZMQ PUSH/PULL | RearRadarR data |
| **5558** | Pi → Jetson | ZMQ PUSH/PULL | IMU samples |
| **5559** | Bidirectional | ZMQ REQ/REP | Control & Discovery |

> **Why separate ports?** Allows independent flow control. If camera backs up, IMU and radar still flow.

---

## 4. Message Wire Format

All messages use: **32-byte header + variable payload**

### 4.1 Message Header (from PiProtocol.hpp)

```cpp
struct PiMessageHeader {
    uint32_t magic;           // 0x50493034 = "PI04"
    uint16_t version;         // 0x0100 = v1.0
    uint16_t msg_type;        // MessageType enum
    uint32_t payload_size;    // Payload bytes after header
    uint64_t timestamp_ns;    // Unix epoch nanoseconds (Chrony-synced!)
    uint32_t sequence;        // Per-stream sequence number
    uint32_t reserved;        // Set to 0
};
// Total: 32 bytes
```

### 4.2 Message Types

```cpp
enum class MessageType : uint16_t {
    REAR_CAM_FRAME  = 0x0001,
    REAR_RADAR_L    = 0x0002,
    REAR_RADAR_R    = 0x0003,
    IMU_SAMPLE      = 0x0004,  // ← NEW: IMU data
    HEARTBEAT       = 0x0010,
    DISCOVERY_REQ   = 0x0020,
    DISCOVERY_RSP   = 0x0021,
};
```

---

## 5. Payload Formats

### 5.1 RearCam Frame (msg_type = 0x0001)

```cpp
struct CameraPayloadHeader {
    uint16_t width;           // e.g., 1280
    uint16_t height;          // e.g., 720
    uint8_t  encoding;        // 1 = MJPEG (recommended)
    uint8_t  reserved[3];
};
// Followed by: MJPEG bytes
```

### 5.2 Radar Data (msg_type = 0x0002, 0x0003)

```cpp
struct RadarPayloadHeader {
    uint16_t data_length;     // Length of raw serial data
    uint8_t  radar_type;      // 0 = OPS243
    uint8_t  reserved;
};
// Followed by: raw serial bytes (DO NOT PARSE - Jetson handles it)
```

### 5.3 IMU Sample (msg_type = 0x0004) - NEW

```cpp
struct ImuPayload {
    // Accelerometer (m/s²)
    float accel_x, accel_y, accel_z;
    
    // Gyroscope (rad/s)
    float gyro_x, gyro_y, gyro_z;
    
    // Magnetometer (µT) - set to 0 if unavailable
    float mag_x, mag_y, mag_z;
    
    // Quaternion orientation (if sensor provides it)
    float quat_w, quat_x, quat_y, quat_z;
    
    // Temperature (°C)
    float temperature;
    
    // Calibration status (0-3 per BNO085)
    uint8_t calibration_status;
    uint8_t reserved[3];
};
// Total: 60 bytes
```

**IMU Sample Rate:** Send at **100 Hz** (every 10ms)

### 5.4 Heartbeat (msg_type = 0x0010)

```cpp
struct HeartbeatPayload {
    uint64_t uptime_ms;           // Pi uptime
    uint8_t  rear_cam_healthy;    // 1=OK, 0=ERROR
    uint8_t  radar_l_healthy;
    uint8_t  radar_r_healthy;
    uint8_t  imu_healthy;         // NEW
    int32_t  chrony_offset_us;    // Clock offset to Jetson
    uint32_t reserved;
};
// Total: 20 bytes
```

---

## 6. Timestamp Protocol

> **CRITICAL:** The Jetson will NOT re-timestamp data from Pi. Your timestamps are authoritative.

### When to timestamp:

```python
# CORRECT - timestamp IMMEDIATELY after read
raw_imu = imu.read()
timestamp = time.time_ns()  # ← RIGHT HERE
send_imu(raw_imu, timestamp)

# WRONG - timestamp before processing
timestamp = time.time_ns()
raw_imu = imu.read()        # ← Delay here!
processed = parse(raw_imu)  # ← More delay!
send_imu(processed, timestamp)
```

---

## 7. Python Implementation Example

```python
import zmq
import struct
import time
import numpy as np

# From PiProtocol.hpp
MAGIC = 0x50493034
VERSION = 0x0100
MSG_REAR_CAM = 0x0001
MSG_RADAR_L = 0x0002
MSG_RADAR_R = 0x0003
MSG_IMU = 0x0004
MSG_HEARTBEAT = 0x0010

class PiSensorPublisher:
    def __init__(self, jetson_ip):
        self.ctx = zmq.Context()
        
        # Connect to Jetson (Jetson binds, Pi connects)
        self.cam_sock = self._create_push(jetson_ip, 5555)
        self.radar_l_sock = self._create_push(jetson_ip, 5556)
        self.radar_r_sock = self._create_push(jetson_ip, 5557)
        self.imu_sock = self._create_push(jetson_ip, 5558)
        
        self.sequences = {MSG_REAR_CAM: 0, MSG_RADAR_L: 0, 
                          MSG_RADAR_R: 0, MSG_IMU: 0}
    
    def _create_push(self, ip, port):
        sock = self.ctx.socket(zmq.PUSH)
        sock.connect(f"tcp://{ip}:{port}")
        sock.setsockopt(zmq.SNDHWM, 10)  # Limit queue (drop old)
        return sock
    
    def _build_header(self, msg_type, payload_size):
        seq = self.sequences[msg_type]
        self.sequences[msg_type] = seq + 1
        timestamp = time.time_ns()
        return struct.pack('<IHHIQII',
            MAGIC, VERSION, msg_type, payload_size,
            timestamp, seq, 0)
    
    def send_imu(self, accel, gyro, mag, quat, temp, calib):
        """Send IMU sample (call at 100Hz)"""
        payload = struct.pack('<fffffffffffff BBB',
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            mag[0], mag[1], mag[2],
            quat[0], quat[1], quat[2], quat[3],
            temp, calib, 0, 0, 0)
        header = self._build_header(MSG_IMU, len(payload))
        self.imu_sock.send(header + payload, zmq.NOBLOCK)
    
    def send_camera_frame(self, jpeg_bytes, width, height):
        """Send MJPEG camera frame"""
        payload_header = struct.pack('<HHBBBB', width, height, 1, 0, 0, 0)
        payload = payload_header + jpeg_bytes
        header = self._build_header(MSG_REAR_CAM, len(payload))
        self.cam_sock.send(header + payload, zmq.NOBLOCK)
    
    def send_radar(self, raw_bytes, is_left):
        """Send raw radar serial data"""
        payload = struct.pack('<HBB', len(raw_bytes), 0, 0) + raw_bytes
        msg_type = MSG_RADAR_L if is_left else MSG_RADAR_R
        sock = self.radar_l_sock if is_left else self.radar_r_sock
        header = self._build_header(msg_type, len(payload))
        sock.send(header + payload, zmq.NOBLOCK)
```

---

## 8. Discovery Protocol

When Jetson runs "Register Pi4 Network Devices":

### Request (Jetson → Pi on port 5559)
```
Header: msg_type=0x0020, payload_size=0
```

### Response (Pi → Jetson)
```cpp
struct DiscoveryResponsePayload {
    uint8_t num_devices;  // 4 (RearCam, RadarL, RadarR, IMU)
    uint8_t reserved[3];
};
// Followed by 4x DeviceInfo structs
```

**Pi should always report these 4 devices:**
```cpp
DeviceInfo devices[4] = {
    {CAMERA, REAR_CAM, OK, "CAM001"},
    {RADAR, REAR_RADAR_L, OK, "RAD001"},
    {RADAR, REAR_RADAR_R, OK, "RAD002"},
    {IMU, IMU, OK, "IMU001"},
};
```

---

## 9. Data Rates Summary

| Stream | Rate | Typical Size | Bandwidth |
|--------|------|--------------|-----------|
| RearCam | 30 fps | ~50 KB/frame | ~12 Mbps |
| RearRadarL | ~20 Hz | ~100 bytes | ~2 Kbps |
| RearRadarR | ~20 Hz | ~100 bytes | ~2 Kbps |
| IMU | 100 Hz | 60 bytes | ~48 Kbps |
| Heartbeat | 1 Hz | 20 bytes | ~160 bps |
| **Total** | - | - | **~12.1 Mbps** |

---

## 10. Testing Checklist

- [ ] Copy `PiProtocol.hpp` to Pi codebase
- [ ] Verify Chrony sync: offset < 5ms
- [ ] Test each stream independently
- [ ] Verify sequence numbers incrementing
- [ ] Verify timestamps are nanosecond epoch
- [ ] Test with `jetson_receiver.exe` on Jetson
- [ ] Verify discovery response is correct

---

## 11. Quick Reference

| Item | Value |
|------|-------|
| **Shared Header** | `include/adas/common/PiProtocol.hpp` |
| **Camera Port** | 5555 |
| **Radar L Port** | 5556 |
| **Radar R Port** | 5557 |
| **IMU Port** | 5558 |
| **Control Port** | 5559 |
| **Magic** | `0x50493034` ("PI04") |
| **Header Size** | 32 bytes |
| **IMU Rate** | 100 Hz |
| **Heartbeat Rate** | 1 Hz |

---

## Appendix: Full Header Byte Layout

```
Offset  Size  Field           Description
─────────────────────────────────────────────────────
0x00    4     magic           0x50493034 ("PI04")
0x04    2     version         0x0100 (v1.0)
0x06    2     msg_type        MessageType enum
0x08    4     payload_size    Payload length in bytes
0x0C    8     timestamp_ns    Unix epoch nanoseconds
0x14    4     sequence        Per-stream counter
0x18    4     reserved        Set to 0
─────────────────────────────────────────────────────
        32 bytes total
```
