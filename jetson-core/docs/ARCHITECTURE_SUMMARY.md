# CU ADAS Architecture Summary

> **Machine-readable companion to** `CU_ADAS_ALL_Views_Iter2.drawio.xml`
>
> This document catalogs every page, element, and connection in the Draw.io
> diagram so that future agents (human or AI) can understand the system without
> opening the visual editor.

---

## Page Index

| # | Diagram Name | View Type | Scope |
|---|---|---|---|
| 1 | `CUADAS_Physical_View_iter1` | Physical / Deployment | Full system hardware topology |
| 2 | `Pi_Process_View` | Process | Raspberry Pi 4 threads |
| 3 | `Jetson_Process_View` | Process | Jetson Nano top-level thread map |
| 4 | `Jetson_PV_CameraIngest_Thread` | Process (detail) | Stage A — Camera capture loop |
| 5 | `Jetson_PV_RadarIngest_Thread` | Process (detail) | Stage A — Radar serial-port loop |
| 6 | `Jetson_PV_NetworkReceiver_Thread` | Process (detail) | Stage A — ZMQ ingestion from Pi |
| 7 | `Jetson_PV_CameraInference_Thread` | Process (detail) | Stage B — TensorRT YOLOv5n inference |
| 8 | `Jetson_PV_StageE_Thread_FULLDETAILS` | Process (detail) | Stage E — Fusion, FCW, BLE encoding |
| 9 | `Jetson_PV_BLE_SideCar` | Process (detail) | BLE peripheral (C++ ↔ Python) |
| 10 | `Mobile_Process_View` | Process | Android app — BLE → UI pipeline |

---

## Page 1 — Physical View (`CUADAS_Physical_View_iter1`)

### Hardware Nodes

| Node | Role | Key Detail |
|---|---|---|
| **Raspberry Pi 4** | Sensor hub | Runs `pi_publisher.py`; collects camera, radar, and IMU data |
| **NVIDIA Jetson Nano** | Main brain | Runs `jetson_core_pipeline`; performs inference, fusion, alerting |
| **Android Device** | Driver UI | Mobile application receives alerts over BLE |

### Sensors

| Sensor | Mount | Interface |
|---|---|---|
| Front Camera | `FrontCam (0)` | CSI / USB |
| Side Camera L | `SideCamL (1)` | Pi-attached |
| Side Camera R | `SideCamR (2)` | Pi-attached |
| Rear Camera | `RearCam (3)` | Pi-attached |
| Front Radar | `FrontRadar (4)` | Serial TTL (UART) |
| Rear-Corner Radar L | `RearCornerRadarL (5)` | Pi-attached |
| Rear-Corner Radar R | `RearCornerRadarR (6)` | Pi-attached |
| IMU | `IMU (7)` | I²C / SPI |

### Communication Links

| From → To | Protocol | Port/Channel | Payload |
|---|---|---|---|
| Pi → Jetson (Camera) | ZMQ PUB/SUB | `tcp://PI_IP:5555` | Camera frames |
| Pi → Jetson (Radar) | ZMQ PUB/SUB | `tcp://PI_IP:5556` | Radar targets |
| Pi → Jetson (IMU) | ZMQ PUB/SUB | `tcp://PI_IP:5557` | IMU samples |
| Pi → Jetson (Heartbeat) | ZMQ PUB/SUB | `tcp://PI_IP:5558` | Heartbeat messages |
| Jetson → Android | BLE GATT Notify | Service `0000ada5-…` / Char `0000a1e7-…` | CBOR-encoded `TickPayload` |

---

## Page 2 — Pi Process View (`Pi_Process_View`)

### Threads

| Thread | Responsibility | Output |
|---|---|---|
| **Camera Thread** | Captures frames from attached cameras | ZMQ publish on `:5555` |
| **Radar L Thread** | Reads left radar via serial | ZMQ publish on `:5556` |
| **Radar R Thread** | Reads right radar via serial | ZMQ publish on `:5556` |
| **IMU Thread** | Reads IMU sensor data | ZMQ publish on `:5557` |
| **Control Thread** | Heartbeat, health monitoring | ZMQ publish on `:5558` |

### Flow

```
Camera  ─┐
Radar L ─┤
Radar R ─┼──► ZMQ PUB sockets ──► Jetson SUB
IMU     ─┤
Control ─┘
```

---

## Page 3 — Jetson Process View (`Jetson_Process_View`)

### Top-Level Architecture

```
                    ┌─── Stage A ───┐
                    │ CameraIngest  │──► cam_front_queue (SPSC 8)
                    │ RadarIngest   │──► radar_front_queue (SPSC 8)
                    │ NetworkRecv   │──► remote queues
                    │ IMU Ingest    │──► imu_queue
                    └───────────────┘
                           │
                    ┌─── Stage B ───┐
                    │ CameraInfer.  │──► g_det_front_queue (SPSC 8)
                    └───────────────┘
                           │
                    ┌─── Stage E ───┐
                    │ Visualization │  (fusion + FCW + BLE + display)
                    │ Thread        │──► BLE GATT notify
                    └───────────────┘
```

### Orchestrator Responsibilities

- Spawns all threads
- Manages `g_running` atomic flag
- Signal handler for graceful shutdown (`SIGINT`, `SIGTERM`)
- Debug controls: `g_visualizer_enabled` flag

---

## Page 4 — Camera Ingest Thread (`Jetson_PV_CameraIngest_Thread`)

### Flow

1. **Open camera** — GStreamer pipeline or V4L2 device
2. **Configure** — Resolution, FPS, codec
3. **Main loop** (`while g_running`):
   - `cap.read(frame)` — blocks until frame available
   - Check `frame.empty()` → skip if bad
   - Stamp `Header` with `CLOCK_MONOTONIC_RAW` as `t_ingest_ns`
   - Set `mount = Mount::FrontCam`, increment `seq`
   - `cam_front_queue_.try_push(CameraFrameData{header, frame})`
   - If queue full → **drop** (freshness over completeness)
4. **Cleanup** — Release camera

### Key Data Structures

- **Input:** Raw camera device
- **Output:** `CameraFrameData` → `SPSCQueue<CameraFrameData, 8> cam_front_queue_`
- **Design:** Non-blocking push; newest frame always preferred

---

## Page 5 — Radar Ingest Thread (`Jetson_PV_RadarIngest_Thread`)

### Flow

1. **Open serial port** — `/dev/ttyUSBx`, baud rate configured
2. **Configure** — UART settings (8N1)
3. **Main loop** (`while g_running`):
   - `read()` from serial port — blocks
   - Parse radar frame (vendor-specific binary protocol)
   - Extract targets: `range_m`, `velocity_mps`, `azimuth_rad`
   - Build `RadarTargets` with `Header` (timestamp, mount, seq)
   - `radar_queue_.try_push(targets)`
4. **Cleanup** — Close serial port

### Key Data Structures

- **Input:** Serial byte stream
- **Output:** `RadarTargets` → `SPSCQueue<RadarTargets, 8>`

---

## Page 6 — Network Receiver Thread (`Jetson_PV_NetworkReceiver_Thread`)

### Flow

1. **Create ZMQ SUB sockets** — one per channel:
   - Camera: `:5555`
   - Radar: `:5556`
   - IMU: `:5557`
   - Heartbeat: `:5558`
2. **Subscribe** — Empty filter (all messages)
3. **Main loop** (`while g_running`):
   - `zmq_poll()` across all sockets
   - On Camera data: deserialize → push to camera queue
   - On Radar data: deserialize → push to radar queue
   - On IMU data: deserialize → push to IMU queue
   - On Heartbeat: update health status, log timing
4. **Cleanup** — Close ZMQ sockets

### Key Design

- Uses `zmq_poll` for efficient multiplexing
- Each message includes `NetPacketHeader` with `Mount` + timestamp
- Remote sensor data gets same `Header` treatment as local sensors

---

## Page 7 — Camera Inference Thread (`Jetson_PV_CameraInference_Thread`)

### Flow

1. **Enter loop** (`while g_running`):
   - `input_queue_.try_pop(frame_data)` — non-blocking drain
   - If empty → `sleep_for(1ms)` → retry
   - If got frame with `frame.empty()` → skip
2. **Preprocessing:**
   - `cv::resize` to 640×640 (YOLO input size)
   - `cv::cvtColor` BGR→RGB
   - `convertTo` CV_32FC3, scale `1.0/255.0`
   - HWC→CHW transposition via `cv::split` + `memcpy`
3. **GPU Transfer:**
   - `cudaMemcpy` input tensor → GPU (`HostToDevice`)
4. **Inference:**
   - `context_->executeV2(bindings)` — TensorRT YOLOv5n
5. **GPU Read-back:**
   - `cudaMemcpy` output tensor → CPU (`DeviceToHost`)
6. **Post-processing:**
   - Parse YOLO output: `num_anchors × 85` (cx, cy, w, h, obj_conf, 80 class scores)
   - For each anchor:
     - Filter `obj_conf < threshold` → skip
     - `argmax(class_scores)` → best class
     - Filter `obj_conf × class_score < threshold` → skip
     - **ADAS class filter:** only keep `0=person, 1=bicycle, 2=car, 3=motorcycle, 5=bus, 7=truck`
     - Scale box coordinates back to original frame dimensions
   - `cv::dnn::NMSBoxes` — Non-Maximum Suppression
   - Convert to `Det` structs
   - Cache: `last_dets`, `last_inference_time`
7. **Output:**
   - `output_queue_.try_push(DetBatch)` — if full, **DROPPED**
   - Design: **freshness over completeness**

### Key Data Structures

- **Input:** `SPSCQueue<CameraFrameData, 8> cam_front_queue_`
- **Output:** `DetBatch` → `SPSCQueue<DetBatch, 8> g_det_front_queue`
- **Model:** YOLOv5n via TensorRT (640×640 input)

---

## Page 8 — Stage E Full Details (`Jetson_PV_StageE_Thread_FULLDETAILS`)

This page contains **three sub-diagrams** and the full **visualization thread** flow.

### Sub-Diagram A: `SensorFusion::fuse` Internals

```
camera dets AND radar targets?
├── either empty → Return empty (no fusion possible)
└── both present → For each camera detection:
    1. pixelToAzimuth(centroid.x)
       normalized = (center − pixel_x) / width
       azimuth = normalized × cam_fov_h_rad
    2. findRadarMatch (1D radar: closest-range target within ±30° of center)
    3. radar match found?
       ├── yes → FusedObject: has_radar=true
       │         range_m, velocity_mps
       │         ttc_s = range / velocity
       │         sources = SRC_CAM_F | SRC_RAD_F
       └── no  → FusedObject: has_radar=false
                 range=0, ttc=∞, camera-only
    4. Add to fused vector
```

### Sub-Diagram B: `FCWMonitor::check` Internals

```
For each fused object with has_radar:
  1. isRelevantClass? (person, bicycle, car, motorcycle, bus, truck)
     └── no → Skip object
  2. Range within [min, max] bounds?
     └── no → Skip object
  3. Trigger 1: TTC below threshold?
     └── yes → Alert candidate
  4. Trigger 2: Physics-based check
     stopping_dist = v² / (2μg) + v × t_react
     range < stopping_dist?
     └── yes → Alert candidate (physics_triggered=true)
     └── no  → Skip object
  5. More urgent than current best?
     └── yes → Update most_urgent:
               ttc_s, range_m, velocity_mps,
               object_class, timestamp_ns, physics_triggered
```

### Sub-Diagram C: BLE Pipeline

```
fcw_alert?
├── yes → FCWAlertAdapter::convert → Alert(type=FCW, severity from TTC)
└── no  → proximity_alert? (any fused object with range < 1.5m)
          ├── yes → Synthetic Alert(type=FCW, severity=Critical, rationale="Proximity Warning 1.5m")
          └── no  → Heartbeat: empty alerts, speed=0
                    ↓
    encodeTickPayloadToCbor(tickId, speed_kmh, heading, lat, alerts)
    Output: CBOR byte vector
                    ↓
    fragmentPayload(tickId, payload, MTU=185)
    Each frame: 4-byte header + CBOR slice
    Header: tickId_lo, tickId_hi, seq_no, seq_max
                    ↓
    for each frame:
      notifyAlertStream(frame)
      sleep 20ms between fragments
      Writes to stdin pipe of ble_peripheral.py
```

### Visualization Thread (Main Loop)

```
visualizationThread
  └── g_visualizer_enabled?
      ├── false → BUG: exits thread (documented issue)
      └── true  → cv::namedWindow("Stage B: FrontCam")
          └── while running AND not shutdown:
              Step 1:  Drain detection queue (keeps LATEST only)
              Step 2:  Drain radar queue (keeps LATEST only)
              Step 3:  Drain IMU queue (all samples)
              Step 4:  EgoFrame::update(imu_sample, dt) — Kalman filter
              Step 5:  fcw_monitor->setEgoVelocity(ego_frame->getForwardVelocity_mps)
              Step 6:  got_frame AND frame not empty?
                       ├── no  → skip to cv::waitKey
                       └── yes:
                            Step 4:  SensorFusion::fuse(batch, radar)
                            Step 5:  Velocity deadband (|vel| < 1.0 → clamp to 0)
                            Step 6:  FCWMonitor::check(fused, now_ns)
                            Step 7:  Proximity alert check (range < 1.5m)
                            Step 8:  Update FCW hold timer (now + 2s)
                            Step 9:  g_ble_server connected?
                                     ├── no  → skip
                                     └── yes → is_alerting OR 1s since last send?
                                               ├── yes → Build alert list → send
                                               └── no  → skip
                            Step 10: Metrics logging (e2e latency, FPS, TTC, range)
                            Step 11: vis = batch.frame.clone() (own copy)
                            Step 12: Persistent detection buffer (500ms TTL)
                            Step 13: Draw bounding boxes, labels, centroids
                            Step 14: now < fcw_alert_until?
                                     └── yes → Red border 8px + "FCW ALERT Range X.Xm"
                            Step 15: Draw info overlay (Inf XXms, FPS XX)
                            Step 16: display_interval 50ms elapsed?
                                     └── yes → cv::imshow
              cv::waitKey(1) — 'q' pressed → shutdown
              no frame received? → sleep 5ms
          └── loop ends → cv::destroyWindow → Thread exits
```

---

## Page 9 — BLE SideCar (`Jetson_PV_BLE_SideCar`)

### Architecture: C++ Parent ↔ Python Child

```
┌─────────────────────────────┐     stdin pipe (JSON)     ┌─────────────────────────┐
│  C++ SimpleBleServer        │ ──────────────────────► │  Python ble_peripheral.py │
│  (part of jetson_core)      │                           │  (child process)          │
└─────────────────────────────┘                           └─────────────────────────┘
```

### C++ Side: `SimpleBleServer`

**Initialization:**
1. `SimpleBleServer::initialize`
2. Log Service UUID `0000ada5-…` and AlertStream UUID `0000a1e7-…`
3. `startAdvertising`
4. `launchPythonBle` → fork + exec
5. `connected_ = true`, `mtu_ = 185`

**Process Spawn (fork/exec):**
1. `pipe(pipefd)` — create Unix pipe
2. `fork()` →
   - **Child (pid == 0):**
     - Close write end of pipe
     - `dup2(read_end, STDIN_FILENO)`
     - `execlp("python3", "scripts/ble_peripheral.py")` — replaces process image
   - **Parent:**
     - Close read end of pipe
     - `python_stdin = fdopen(write_end, "w")`
     - `setlinebuf(python_stdin)`

**`notifyAlertStream` (called from Stage E):**
1. Receive `vector<uint8_t>` (one BLE fragment)
2. Convert binary to hex string (`0xAB 0xCD → "abcd"`)
3. Wrap in JSON: `{"cmd": "notify", "data": "<hex>"}`
4. `lock_guard(write_mutex)` → `fprintf(python_stdin, JSON\n)` → `fflush`

**Shutdown:**
1. `fclose(python_stdin)` — Python sees EOF
2. `kill(python_pid, SIGTERM)`
3. `waitpid(python_pid)` — reap child, prevent zombie

### Python Side: `ble_peripheral.py`

**Setup (`BlePeripheral.run`):**
1. `find_adapter` via D-Bus (scan BlueZ for GATT manager)
2. Create Application → `/org/bluez/adas`
3. Create Service → UUID `0000ada5-…`, `primary=true`
4. Create Characteristic → UUID `0000a1e7-…`, flags: `notify`
5. `gatt_manager.RegisterApplication`
6. `ad_manager.RegisterAdvertisement` → advertise as "ADAS-Jetson"

**Runtime:**
- Start `stdin_reader_thread` (daemon)
- `GLib.MainLoop.run()` — D-Bus event loop (blocks main thread)
- **stdin_reader_thread** loop:
  1. `line = sys.stdin.readline()` — blocks until C++ writes
  2. EOF? → `mainloop.quit()`, `running = False`, thread exits
  3. Else: `GLib.idle_add(process_stdin_command, line)` → dispatches to main thread

**`process_stdin_command`:**
1. `json.loads(line)`
2. `cmd == "notify"`?
   - **yes:** `bytes.fromhex(hex_data)` → `alert_chrc.send_notification(data)` → `PropertiesChanged` D-Bus signal → BlueZ daemon → BLE radio → Mobile App `onCharacteristicChanged`
   - **no:** Log "Unknown command"

---

## Page 10 — Mobile Process View (`Mobile_Process_View`)

### Lane 1: BLE Reception (Binder Thread)

**Connection Sequence:**
1. `MainActivity.onCreate` → `BleManager(context)`
2. `bleManager.initialize` — get `BluetoothAdapter` + `LeScanner`
3. `scanForJetson`:
   - `ScanFilter: ADAS_SERVICE_UUID (0000ada5-…)`
   - `ScanMode: LOW_LATENCY`
   - `startScan`, schedule 5s timeout
4. On scan result: `stopScan` → `connectToJetson(device)`
   - `device.connectGatt(transport = TRANSPORT_LE)`
5. `onConnectionStateChange(STATE_CONNECTED)`:
   - `gatt.discoverServices`
6. `onServicesDiscovered`:
   - `getService(ADAS_SERVICE_UUID)`
   - `getCharacteristic(ADAS_ALERT_STREAM_UUID, 0000a1e7-…)`
   - `setCharacteristicNotification(true)`
   - `writeDescriptor(CCCD, ENABLE_NOTIFICATION)`
   - `gatt.requestMtu(185)`
7. `onMtuChanged: currentMtu = mtu`

**Scan Retry:** Timeout 5s → `stopScanAndRetry` → `scheduleRetry: delay 2s` → loop

**Disconnection:** `onConnectionStateChange(STATE_DISCONNECTED)` → `bluetoothGatt = null` → TODO: `scheduleRetry` (not implemented)

**Fragment Reception Loop:**
1. `onCharacteristicChanged` (Binder thread callback)
2. `BLEHeader.fromBytes(data)` — little-endian: `tickId(uint16)`, `seqNo(uint8)`, `seqMax(uint8)`
3. `slice = data[4:]` — payload fragment
4. `activeTick.tickId == header.tickId`?
   - **no:** new tick → `activeTick = InProgressTick(tickId, seqMax, fragments_map)`
   - **yes:** same tick
5. `fragments[seqNo] = slice`, `receivedCount++`
6. `receivedCount == seqMax + 1`?
   - **no:** wait for more fragments
   - **yes:**
     1. `rebuildFullPayload` — concatenate fragments 0..seqMax in order
     2. `TickDecoder.decode` (debug log: tickId, speed, healthMask, alerts)
     3. `_packetFlow.tryEmit(cborBuffer)` — `SharedFlow(buffer=64, DROP_OLDEST)`
     4. `activeTick = null` → loop

### Lane 2: Data Processing (Dispatchers.Default Coroutine)

```
BleTickRepository
  └── blePackets flow received
      └── merge:
          1. blePackets.map { TickDecoder.decode(it) }
          2. debugFlow (local heartbeat at 1Hz)
      └── TickDecoder.decode(bytes)
          Cbor(ignoreUnknownKeys=true) → TickPayload
          (tickId, speed, healthMask, bsdMask, alerts)
      └── runningFold { VehicleAlertReducer.reduce(state, tick) }
```

**`VehicleAlertReducer.reduce`:**
1. `tickId >= 0 AND tickId <= prev.lastTickId?` → **yes: DROP** (out-of-order)
2. `healthFromMask(tick.healthMask)` → `CameraHealth(frontOk, rearOk)` + `RadarHealth(ok)`
3. `bsdFromMask(tick.bsdMask)` → `BlindSpotStatus(left, right)`
4. Tick has FCW alert? (`type == 0`)
   - **yes:** `fcwExpiry = now + 3000ms` (3-second latch hold)
5. `now < fcwExpiry?`
   - **yes (LATCHED):** `sonar.front = RED`, detection = Car with confidence from severity
   - **no (EXPIRED):** `sonar.front = GREEN`, detection = None
6. Return `VehicleAlert(cameras, radar, bsd, telemetry, sonar, detection, activeAlerts, lastTickId, fcwExpiry)`

**Output:** `dashboardState: StateFlow<VehicleAlert>` — `stateIn(WhileSubscribed, 5000)`

**Heartbeat Coroutine (1 Hz):**
- `while true` → `delay(1000ms)` → `debugFlow.emit(TickPayload(tickId=-1, …))` — bypasses ordering check, triggers latch expiry re-evaluation

### Lane 3: UI Rendering (Main Thread)

```
DriveScreen Composable
  └── val vehicleAlert by repository.dashboardState.collectAsState()
      └── Automatic recomposition on state change
          ├── Speed display:       vehicleAlert.telemetry.speedKmh
          ├── Sonar indicators:    front / rear / left / right  (GREEN / YELLOW / RED / OFF)
          ├── Camera health:       frontOk / rearOk
          ├── Radar health:        ok
          ├── BSD indicators:      leftActive / rightActive
          └── FCW alert overlay:   ObjectDetection.Car or None
```

---

## Cross-Cutting Concerns

### Thread-Safe Queues

All inter-thread communication uses **lock-free SPSC (Single-Producer, Single-Consumer) queues** with a capacity of **8 slots**.

| Queue | Producer | Consumer | Payload |
|---|---|---|---|
| `cam_front_queue_` | CameraIngest | CameraInference | `CameraFrameData` |
| `radar_front_queue_` | RadarIngest | Visualization | `RadarTargets` |
| `imu_queue_` | IMU Ingest | Visualization | `ImuSample` |
| `g_det_front_queue` | CameraInference | Visualization | `DetBatch` |

### Design Principle: Freshness Over Completeness

When a queue is full, new data is **dropped** rather than blocking the producer. This ensures the pipeline always processes the most recent sensor data, a critical requirement for real-time safety applications.

### Timestamps

- `t_ingest_ns` (`CLOCK_MONOTONIC_RAW`) is the **authoritative** timestamp
- `t_device_ns` is from the sensor device clock (may be 0 if unavailable)

### Known Issues (from diagram annotations)

- **`g_visualizer_enabled = false` BUG:** If the visualizer flag is off, the entire Stage E thread exits immediately, disabling all fusion, FCW, and BLE logic — not just the display window
