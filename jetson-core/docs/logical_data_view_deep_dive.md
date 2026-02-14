# Logical Data View — Deep Dive for 4+1 Architecture

## What Belongs Here

The Logical View shows the system's **key abstractions** — data structures, class relationships, and protocol schemas. It answers "what data flows through the system and what shape does it take?"

This covers what the Process View (threads/concurrency) and Physical View (hardware/deployment) do NOT:

- **Data contracts** — the structs that cross component boundaries
- **Enumerations** — shared vocabularies (Mount, AlertType, ObjectClass)
- **Configuration objects** — tunable parameters
- **Protocol schemas** — BLE wire format, ZMQ wire format, CBOR encoding
- **Cross-boundary contracts** — how C++ data maps to Kotlin data over BLE

---

## Layer 1: Shared Foundation

### Header (every sensor payload carries this)

| Field | Type | Description |
|-------|------|-------------|
| `t_device_ns` | `uint64` | Device clock (0 if unavailable) |
| `t_ingest_ns` | `uint64` | **Authoritative** receive time (`CLOCK_MONOTONIC_RAW`) |
| `mount` | `Mount` | Which physical position (not device brand) |
| `seq` | `uint32` | Per-mount monotonic sequence counter |
| `healthy` | `bool` | Device/ingest health bit |

### Mount Enum (vehicle positions)

```
FrontCam=0  SideCamL=1  SideCamR=2  RearCam=3
FrontRadar=4  RearCornerRadarL=5  RearCornerRadarR=6
IMU=7  LiDAR2D=8(reserved)
```

> [!IMPORTANT]
> Mount represents a **position**, not a device. Devices are swappable (FR16). Algorithms bind to Mount.

### SensorSource Bitmask

```
SRC_CAM_F=0x01  SRC_CAM_L=0x02  SRC_CAM_R=0x04  SRC_CAM_B=0x08
SRC_RAD_F=0x10  SRC_RAD_L=0x20  SRC_RAD_R=0x40
SRC_IMU=0x80    SRC_LIDAR=0x100(reserved)
```

Used in `FusedObject.sources` and `Alert.sources` for provenance tracking.

---

## Layer 2: Sensor Payloads (Stage A Output)

### CameraFrameData

| Field | Type | Description |
|-------|------|-------------|
| `h` | `Header` | Mount, timestamps, seq |
| `width` | `int` | Frame width (pixels) |
| `height` | `int` | Frame height (pixels) |
| `channels` | `int` | Always 3 (BGR) |
| `data` | `vector<uint8>` | Raw pixel bytes (optional) |
| `frame` | `cv::Mat` | OpenCV Mat (zero-copy path) |

**Queues:** `SPSCQueue<CameraFrameData, 8>` × 4 (front, left, right, rear)

### RadarTarget (single detection)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `range_m` | `float` | meters | Radial distance |
| `azimuth_rad` | `float` | radians | Angle (+left) |
| `radial_vel_mps` | `float` | m/s | Toward=negative, away=positive |
| `rcs_db` | `float` | dBsm | Radar cross section |
| `sigma_r/az/v` | `float` | — | Measurement stdev |

### RadarTargets (batch)

| Field | Type |
|-------|------|
| `h` | `Header` |
| `targets` | `vector<RadarTarget>` |

**Queue:** `SPSCQueue<RadarTargets, 8>` × 1 (front radar)

### ImuSample (high-rate: ≥100 Hz)

| Field | Type | Unit |
|-------|------|------|
| `t_capture` | `uint64` | nanoseconds |
| `accel[3]` | `float[3]` | m/s² (ax, ay, az) |
| `gyro[3]` | `float[3]` | rad/s (wx, wy, wz) |
| `mag[3]` | `float[3]` | µT (optional) |
| `quat[4]` | `float[4]` | w, x, y, z (optional) |
| `temperature` | `float` | °C |
| `calibration_status` | `uint8` | BNO085: 0-3 |

**Queue:** `SPSCQueue<ImuSample, 32>` × 1

---

## Layer 3: Inference Output (Stage B Output)

### ObjectClass Enum (COCO subset)

```
Person=0  Bicycle=1  Car=2  Motorcycle=3  Bus=5  Truck=7  Unknown=255
```

### Det (single detection)

| Field | Type | Description |
|-------|------|-------------|
| `box_px` | `cv::Rect2f` | x, y, w, h in pixels |
| `centroid` | `cv::Point2f` | Box center (for fusion) |
| `cls` | `int` | ObjectClass value |
| `score` | `float` | Confidence [0, 1] |

### DetBatch (frame of detections)

| Field | Type | Description |
|-------|------|-------------|
| `h` | `Header` | Inherited from source camera frame |
| `dets` | `vector<Det>` | All detections in frame |
| `inference_time_us` | `uint64` | Inference latency (µs) |
| `frame` | `cv::Mat` | Frame for visualization |

**Queue:** `SPSCQueue<DetBatch, 8>` × 1

---

## Layer 4: Fusion Output (Stage E Internal)

### FusedObject

| Field | Type | Source | Description |
|-------|------|--------|-------------|
| `track_id` | `uint32` | Camera | Tracker ID (if available) |
| `object_class` | `int` | Camera | ObjectClass value |
| `score` | `float` | Camera | Detection confidence |
| `box_px` | `cv::Rect2f` | Camera | Bounding box |
| `centroid_px` | `cv::Point2f` | Camera | Box center |
| `cam_azimuth_rad` | `float` | Computed | Angular bearing from pixel position |
| `has_radar` | `bool` | Fusion | True if radar matched |
| `range_m` | `float` | Radar | Distance (0 if no radar) |
| `radial_vel_mps` | `float` | Radar | Velocity (0 if no radar) |
| `radar_azimuth_rad` | `float` | Radar | Radar angle |
| `ttc_s` | `float` | Computed | Time-to-collision (∞ if not approaching) |
| `sources` | `uint16` | Bitmask | SensorSource flags |

### FusionConfig

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cam_fov_h_rad` | 60° | Horizontal FOV |
| `cam_width_px` | 1280 | Frame width |
| `azimuth_match_threshold_rad` | 5° | Match tolerance |
| `min_closing_vel_mps` | 0.5 | TTC noise floor |

---

## Layer 5: Alert Decision (Stage E Internal)

### FCWAlert

| Field | Type | Description |
|-------|------|-------------|
| `ttc_s` | `float` | Time-to-collision |
| `range_m` | `float` | Distance to threat |
| `velocity_mps` | `float` | Closing velocity |
| `object_class` | `int` | Threat type |
| `timestamp_ns` | `uint64` | Generation time |
| `physics_triggered` | `bool` | Whether stopping distance check triggered this |

### FCWMonitor::Config

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ttc_threshold_s` | 3.0 | TTC alert threshold |
| `min_range_m` | 0.5 | Ignore closer (already hit) |
| `max_range_m` | 50.0 | Ignore farther |
| `friction_coefficient` | 0.7 | Road surface (0.7=dry, 0.4=wet, 0.2=ice) |
| `reaction_time_s` | 2.5 | Driver reaction time |
| `use_physics_fcw` | false | Enable physics-based stopping distance check |

### EgoFrame (Kalman Filter)

| Dimension | State Vector | Measurement |
|-----------|-------------|-------------|
| State (5D) | x, y, vx, vy, yaw | — |
| Measurement (4D) | x, y, vx, vy | From IMU accel + quat |

Primary output: `getForwardVelocity_mps()` → feeds into `FCWMonitor.setEgoVelocity()`

---

## Layer 6: Alert Schema (BLE Output)

### Alert (C++ full schema, FR70)

| Field | Type | Description |
|-------|------|-------------|
| `t_ms` | `uint64` | Wallclock timestamp |
| `id` | `string` | Unique ID (e.g. `fcw-{tick}-{class}`) |
| `type` | `AlertType` | LDW=0, FCW=1, RCW=2, BSD=3 |
| `direction` | `optional<string>` | "front" or "rear" |
| `severity` | `Severity` | Info=0, Warning=1, Critical=2 |
| `ttl_ms` | `uint32` | Expiry window |
| `rationale` | `string` | JSON: `{"ttc_s":X,"range_m":Y}` |
| `object_id` | `optional<uint32>` | Track ID |
| `sources` | `vector<string>` | e.g. ["FrontCam","FrontRadar"] |
| `schemaVersion` | `string` | "v1.0" |
| `confidence` | `float` | [0.0, 1.0] |

### FCWAlertAdapter Conversion (FCWAlert → Alert)

```
Severity mapping:
  TTC < 1.0s → Critical
  TTC < 2.0s → Warning
  TTC ≥ 2.0s → Info
```

---

## Layer 7: BLE Wire Protocol

### BLE Frame (on the wire)

```
┌──────────────────────────────────────┐
│ BLEHeader (4 bytes, little-endian)   │
│  tick_id : uint16  (wraps mod 65536) │
│  seq_no  : uint8   (0..seq_max)     │
│  seq_max : uint8   (total frags - 1)│
├──────────────────────────────────────┤
│ CBOR Slice (variable, up to MTU-7)  │
│  Portion of TickPayload CBOR bytes  │
└──────────────────────────────────────┘
```

**Fragmentation:** `slice_cap = ATT_MTU - 3 (ATT overhead) - 4 (our header)`

### TickPayload CBOR Schema (cross-boundary contract)

```
C++ encodes (nlohmann::json → CBOR):      Kotlin decodes (@Serializable):

{ "t": tickId }           ←→    @SerialName("t") val tickId: Int
{ "v": speedKmh }         ←→    @SerialName("v") val speed: Int
{ "h": healthMask }       ←→    @SerialName("h") val healthMask: Int
{ "b": bsdMask }           ←→    @SerialName("b") val bsdMask: Int
{ "a": [AlertDto...] }    ←→    @SerialName("a") val alerts: List<AlertDto>
```

### AlertDto CBOR Schema

```
C++ (alertToCompactJson):               Kotlin (AlertDto):

{ "id": alertTypeWireValue }    ←→    @SerialName("id") val type: Int
{ "s":  severityInt }           ←→    @SerialName("s")  val severity: Int
{ "r":  rationaleString }       ←→    @SerialName("r")  val rationale: String
```

**Wire values for alert type:**
```
C++ AlertType enum:     Wire value:     Android meaning:
  LDW = 0         →       1               1 = LDW
  FCW = 1         →       0               0 = FCW
  RCW = 2         →       2               2 = RCW
  BSD = 3         →       3               3 = BSD
```

> [!WARNING]
> The C++ `AlertType` enum values (LDW=0, FCW=1) are **remapped** by `alertTypeToWireValue()` so that FCW=0 on the wire. The Android side treats `type=0` as FCW. This is a critical contract.

### Health Bitmask Schema

```
C++ encodes:                     Android decodes (TickPayloadMapper):
  bit 0 = frontCam broken   →   frontOk = !mask.isBitSet(0)
  bit 1 = rearCam broken    →   rearOk  = !mask.isBitSet(1)
  bit 2 = radar broken      →   ok      = !mask.isBitSet(2)
```

### BSD Bitmask Schema

```
  bit 0 = left active    →   leftActive  = mask.isBitSet(0)
  bit 1 = right active   →   rightActive = mask.isBitSet(1)
```

---

## Layer 8: Android UI State (Post-Decode)

### VehicleAlert (Compose state)

| Field | Type | Source |
|-------|------|--------|
| `cameras` | `CameraHealth(frontOk, rearOk)` | healthMask bits 0-1 |
| `radar` | `RadarHealth(ok)` | healthMask bit 2 |
| `sonar` | `SonarColors(front, rear, left, right)` | FCW latch logic |
| `telemetry` | `VehicleTelemetry(speedKmh)` | `abs(tick.speed).coerceIn(0,300)` |
| `detection` | `ObjectDetection` (sealed: None / Car / Person) | FCW latch logic |
| `bsd` | `BlindSpotStatus(leftActive, rightActive)` | bsdMask bits 0-1 |
| `lastTickId` | `Int` | Strict ordering (drop if ≤ prev) |
| `fcwExpiry` | `Long` | `now + 3000ms` when FCW received |
| `activeAlerts` | `List<AlertDto>` | Raw alerts for debug |

### ObjectDetection (sealed class)

```kotlin
ObjectDetection.None          // No threat
ObjectDetection.Car(confidence)   // FCW active with severity-based confidence
ObjectDetection.Person(confidence) // Person detected (reserved)
```

### SonarColor Enum

```
OFF(Gray)  GREEN(Green)  YELLOW(Yellow)  RED(Red)
```

Driven by FCW latch: `isLatched → RED`, `else → GREEN`

---

## Layer 9: Network Protocol (Pi4 → Jetson via ZMQ)

### NetPacketHeader (24 bytes, packed)

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | `magic` | `uint32` | `0xADA5DA7A` ("ADAS DATA") |
| 4 | `type` | `uint8` | NetPacketType |
| 5 | `reserved` | `uint8` | Padding |
| 6 | `flags` | `uint16` | Reserved |
| 8 | `payload_size` | `uint32` | Payload bytes |
| 12 | `seq` | `uint32` | Per-type sequence |
| 16 | `pi_timestamp` | `uint64` | Pi local time (informational) |

### NetPacketType Enum

```
RearCamera = 0x01    (MJPEG frame)
RearRadarL = 0x02    (Left rear radar)
RearRadarR = 0x03    (Right rear radar)
Heartbeat  = 0xFE    (Keep-alive)
Error      = 0xFF    (Error indication)
```

---

## GATT Service Structure

```
Service: 0000ADA5-0000-1000-8000-00805f9b34fb  (ADAS Service)
  ├─ 0000A1E7-...  AlertStream   [notify]       ← Active: FCW fragments
  ├─ 000057A7-...  Status        [notify/read]   ← Reserved
  ├─ 0000C0AD-...  Command       [write]          ← Reserved
  └─ 0000FA17-...  Pair          [write/notify]   ← Reserved
```
