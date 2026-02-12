# ADAS Interface Contracts

**Version:** 1.0  
**Owner:** Lead (Damon)  
**Last Updated:** Feb 12, 2026

> ⚠️ **Change Protocol:** If you modify any contract in this document, you MUST update both sides of the boundary and notify the team before merging. Mismatched contracts cause silent data corruption.

---

## Table of Contents

1. Shared Vocabulary
2. Sensor Payloads (Stage A Output)
3. Inference Output (Stage B Output)
4. BLE Wire Protocol (Jetson → Phone)
5. Pi → Jetson Network Protocol

---

## 1. Shared Vocabulary

### Mount Identifiers

Mounts are **positions on the vehicle**, not device brands. Algorithms bind to Mount, not hardware.

| Value | Name | Current Hardware |
|-------|------|-----------------|
| 0 | FrontCam | USB Webcam (MJPEG) |
| 1 | SideCamL | — (reserved) |
| 2 | SideCamR | — (reserved) |
| 3 | RearCam | Via Pi4 ZMQ |
| 4 | FrontRadar | OPS243-A (serial) |
| 5 | RearCornerRadarL | — (reserved) |
| 6 | RearCornerRadarR | — (reserved) |
| 7 | IMU | BNO085 via Pi4 ZMQ |
| 8 | LiDAR2D | — (reserved, no implementation) |

### ADAS Object Classes (COCO subset)

| Value | Class | ADAS Relevance |
|-------|-------|----------------|
| 0 | Person | Pedestrian collision |
| 1 | Bicycle | Cyclist collision |
| 2 | Car | Vehicle collision |
| 3 | Motorcycle | Vehicle collision |
| 5 | Bus | Vehicle collision |
| 7 | Truck | Vehicle collision |
| 255 | Unknown | Filtered out |

### Alert Types

> ⚠️ **GOTCHA:** The C++ enum values ≠ wire values. `alertTypeToWireValue()` remaps them!

| Wire Value | Alert | C++ Enum Value | Android `type` field |
|-----------|-------|----------------|---------------------|
| **0** | FCW (Forward Collision) | `AlertType::FCW = 1` | `0 = FCW` |
| **1** | LDW (Lane Departure) | `AlertType::LDW = 0` | `1 = LDW` |
| **2** | RCW (Rear Collision) | `AlertType::RCW = 2` | `2 = RCW` |
| **3** | BSD (Blind Spot) | `AlertType::BSD = 3` | `3 = BSD` |

### Severity Levels

| Value | Level | FCW Trigger |
|-------|-------|-------------|
| 0 | Info | TTC ≥ 2.0s |
| 1 | Warning | TTC < 2.0s |
| 2 | Critical | TTC < 1.0s |

---

## 2. Sensor Payloads (Stage A Output)

Every sensor payload carries a **Header**:

| Field | Type | Description |
|-------|------|-------------|
| `t_device_ns` | uint64 | Device clock (0 if unavailable) |
| `t_ingest_ns` | uint64 | **Authoritative** timestamp (CLOCK_MONOTONIC_RAW) |
| `mount` | Mount (uint8) | Source position |
| `seq` | uint32 | Per-mount sequence counter |
| `healthy` | bool | Ingest health flag |

### CameraFrameData

Produced by: `CameraIngest` (USB) or `NetworkReceiver::cameraThread` (ZMQ)  
Queue: `SPSCQueue<CameraFrameData, 8>`

| Field | Type | Notes |
|-------|------|-------|
| `h` | Header | — |
| `width` | int | Pixels |
| `height` | int | Pixels |
| `channels` | int | Always 3 (BGR) |
| `data` | vector\<uint8\> | Raw pixel bytes (optional path) |
| `frame` | cv::Mat | OpenCV Mat (primary path, zero-copy) |

### RadarTargets

Produced by: `RadarIngest` (serial 921600 baud, OPS243-A)  
Queue: `SPSCQueue<RadarTargets, 8>`

**RadarTarget** (one per detected object):

| Field | Type | Unit | Notes |
|-------|------|------|-------|
| `range_m` | float | meters | Radial distance |
| `azimuth_rad` | float | radians | Always 0 for 1D OPS243-A |
| `radial_vel_mps` | float | m/s | Positive = away, Negative = toward |
| `rcs_db` | float | dBsm | Radar cross section |

### ImuSample

Produced by: `NetworkReceiver::imuThread` (ZMQ from Pi4 BNO085)  
Queue: `SPSCQueue<ImuSample, 32>`

| Field | Type | Unit |
|-------|------|------|
| `t_capture` | uint64 | nanoseconds |
| `accel[3]` | float × 3 | m/s² (ax, ay, az) |
| `gyro[3]` | float × 3 | rad/s (wx, wy, wz) |
| `quat[4]` | float × 4 | Quaternion w, x, y, z (from BNO085) |
| `temperature` | float | °C |
| `calibration_status` | uint8 | 0–3 |

---

## 3. Inference Output (Stage B Output)

### DetBatch

Produced by: `CameraPipeline::threadFunc`  
Queue: `SPSCQueue<DetBatch, 8>`

| Field | Type | Notes |
|-------|------|-------|
| `h` | Header | Inherited from source CameraFrameData |
| `dets` | vector\<Det\> | All detections in frame |
| `inference_time_us` | uint64 | TensorRT inference latency (µs) |
| `frame` | cv::Mat | Raw frame (undistortion disabled) |

**Det** (one per detected object):

| Field | Type | Notes |
|-------|------|-------|
| `box_px` | cv::Rect2f | x, y, w, h in pixels |
| `centroid` | cv::Point2f | Box center (computed) |
| `cls` | int | ObjectClass value (see §1) |
| `score` | float | Confidence [0.0, 1.0] |

---

## 4. BLE Wire Protocol (Jetson → Phone)

### GATT Service

| UUID | Name | Properties | Status |
|------|------|-----------|--------|
| `0000ADA5-0000-1000-8000-00805f9b34fb` | ADAS Service | — | Active |
| `0000A1E7-0000-1000-8000-00805f9b34fb` | AlertStream | Notify | **Active** |
| `000057A7-0000-1000-8000-00805f9b34fb` | Status | Notify/Read | Reserved |
| `0000C0AD-0000-1000-8000-00805f9b34fb` | Command | Write | Reserved |
| `0000FA17-0000-1000-8000-00805f9b34fb` | Pair | Write/Notify | Reserved |

**Device Name:** `ADAS-Jetson`  
**Negotiated MTU:** 185 bytes

### BLE Frame Format

Each BLE notification contains one frame:

```
Offset  Size   Field       Description
──────  ─────  ──────────  ────────────────────────────────
0       2      tick_id     uint16, little-endian, wraps at 65536
2       1      seq_no      uint8, fragment index (0..seq_max)
3       1      seq_max     uint8, total fragments - 1
4       var    cbor_slice  Portion of CBOR payload
```

**Fragmentation:** `slice_capacity = MTU - 3 (ATT overhead) - 4 (header) = 178 bytes`

### CBOR TickPayload Schema

This is the **central cross-boundary contract**. C++ encodes using `nlohmann::json::to_cbor()`, Android decodes using `kotlinx.serialization.cbor`.

| CBOR Key | C++ Source | Kotlin Field | Type | Description |
|----------|-----------|-------------|------|-------------|
| `"t"` | tickId | `tickId: Int` | int | Tick counter (20 Hz resolution) |
| `"v"` | speedKmh | `speed: Int` | int | Vehicle speed in km/h |
| `"h"` | healthMask | `healthMask: Int` | int | Sensor health bitmask (see below) |
| `"b"` | bsdMask | `bsdMask: Int` | int | Blind spot bitmask (see below) |
| `"a"` | alerts[] | `alerts: List<AlertDto>` | array | Alert list (see below) |

### AlertDto Schema (inside `"a"` array)

| CBOR Key | C++ Source | Kotlin Field | Type | Description |
|----------|-----------|-------------|------|-------------|
| `"id"` | `alertTypeToWireValue(type)` | `type: Int` | int | Alert type (0=FCW, 1=LDW, 2=RCW, 3=BSD) |
| `"s"` | `(int)severity` | `severity: Int` | int | 0=Info, 1=Warning, 2=Critical |
| `"r"` | `rationale` | `rationale: String` | string | JSON: `{"ttc_s":X,"range_m":Y}` |

### Health Bitmask (`"h"` field)

| Bit | Meaning | Android Decode |
|-----|---------|---------------|
| 0 | Front camera broken | `frontOk = !bit0` |
| 1 | Rear camera broken | `rearOk = !bit1` |
| 2 | Radar broken | `ok = !bit2` |

**Convention:** Bit set = sensor is **broken** (inverted on Android side).

### BSD Bitmask (`"b"` field)

| Bit | Meaning | Android Decode |
|-----|---------|---------------|
| 0 | Left blind spot active | `leftActive = bit0` |
| 1 | Right blind spot active | `rightActive = bit1` |

### Transmission Cadence

| Condition | Rate | Content |
|-----------|------|---------|
| FCW/proximity alert active | Immediate | TickPayload with alerts |
| No alert, BLE connected | 1 Hz | Heartbeat (empty alerts, speed=0) |
| BLE disconnected | — | No transmission |

**Fragment pacing:** 20 ms sleep between fragments to avoid BLE congestion.

---

## 5. Pi → Jetson Network Protocol

Transport: **ZeroMQ** (SUB/PUB per topic, REQ/REP for heartbeat)

### ZMQ Topics

| Topic | Data | Queue Target |
|-------|------|-------------|
| `rear_camera` | MJPEG frame (decoded on Jetson) | `cam_rear_queue_` |
| `imu` | BNO085 packed binary | `imu_queue_` |
| `radar_l` | Left rear radar (TODO) | — |
| `radar_r` | Right rear radar (TODO) | — |
| Heartbeat | REQ/REP + Chrony offset | Connection status |

### IMU Wire Format (ZMQ)

Packed binary from Pi4 `imu_publisher.py`:

```
Offset  Size   Field
──────  ─────  ────────────────
0       4×3    accel (3 floats, m/s²)
12      4×3    gyro (3 floats, rad/s)
24      4×3    mag (3 floats, µT)
36      4×4    quaternion (4 floats, w,x,y,z)
52      4      temperature (float, °C)
56      1      calibration_status (uint8, 0-3)
```

---

## Quick Reference: Data Flow Chain

```
CameraFrameData → DetBatch → FusedObject → FCWAlert → Alert
                                                        ↓
                                            alertToCompactJson()
                                                        ↓
                                          encodeTickPayloadToCbor()
                                                        ↓
                                             fragmentPayload()
                                                        ↓
                                              BLE Notification
                                                        ↓
                                          handleIncomingFragment()
                                                        ↓
                                            TickDecoder.decode()
                                                        ↓
                                       VehicleAlertReducer.reduce()
                                                        ↓
                                              VehicleAlert
                                           (Compose UI State)
```
