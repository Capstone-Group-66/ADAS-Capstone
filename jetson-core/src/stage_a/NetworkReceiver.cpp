// File: src/stage_a/NetworkReceiver.cpp
// ZMQ-based receiver for Pi4 sensor data
#include "adas/stage_a/NetworkReceiver.hpp"
#include "adas/common/Clock.hpp"
#include "adas/common/Globals.hpp"
#include "adas/main_brain/Alert.hpp"
#include "adas/recording/Recorder.hpp"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cstring>
#include <iostream>

#include <zmq.h>

using namespace adas::protocol;

namespace adas {

NetworkReceiver::NetworkReceiver(const std::string &pi_ip) : pi_ip_(pi_ip) {
  context_ = zmq_ctx_new();
  if (!context_) {
    throw std::runtime_error("Failed to create ZMQ context");
  }
}

NetworkReceiver::~NetworkReceiver() {
  stop();
  if (context_) {
    zmq_ctx_term(context_);
    context_ = nullptr;
  }
}

std::string NetworkReceiver::buildAddr(int port) const {
  return "tcp://" + pi_ip_ + ":" + std::to_string(port);
}

bool NetworkReceiver::start(SPSCQueue<Alert, 8> *cam_queue,
                            SPSCQueue<ImuSample, 32> *imu_queue,
                            SPSCQueue<RadarTargets, 8> *radar_l_queue,
                            SPSCQueue<RadarTargets, 8> *radar_r_queue) {
  if (running_.load()) {
    return true; // Already running
  }

  cam_queue_ = cam_queue;
  imu_queue_ = imu_queue;
  radar_l_queue_ = radar_l_queue;
  radar_r_queue_ = radar_r_queue;

  // Create sockets
  cam_socket_ = zmq_socket(context_, ZMQ_PULL);
  radar_l_socket_ = zmq_socket(context_, ZMQ_PULL);
  radar_r_socket_ = zmq_socket(context_, ZMQ_PULL);
  imu_socket_ = zmq_socket(context_, ZMQ_PULL);
  heartbeat_socket_ =
      zmq_socket(context_, ZMQ_REQ); // REQ/REP pattern for control

  // Set socket options
  int hwm = 10;
  int timeout = 200; // 200ms receive timeout

  auto configure_socket = [&](void *sock) {
    zmq_setsockopt(sock, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(sock, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
  };

  configure_socket(cam_socket_);
  configure_socket(radar_l_socket_);
  configure_socket(radar_r_socket_);
  configure_socket(imu_socket_);
  configure_socket(heartbeat_socket_);

  // Connect to Pi (Pi binds, we connect)
  std::cout << "[NetworkReceiver] Connecting to Pi at " << pi_ip_ << "...\n";

  if (zmq_connect(cam_socket_, buildAddr(PORT_REAR_CAM).c_str()) != 0) {
    std::cerr << "[NetworkReceiver] Failed to connect camera socket\n";
    return false;
  }
  if (zmq_connect(radar_l_socket_, buildAddr(PORT_RADAR_L).c_str()) != 0) {
    std::cerr << "[NetworkReceiver] Failed to connect radar L socket\n";
    return false;
  }
  if (zmq_connect(radar_r_socket_, buildAddr(PORT_RADAR_R).c_str()) != 0) {
    std::cerr << "[NetworkReceiver] Failed to connect radar R socket\n";
    return false;
  }
  if (zmq_connect(imu_socket_, buildAddr(PORT_IMU).c_str()) != 0) {
    std::cerr << "[NetworkReceiver] Failed to connect IMU socket\n";
    return false;
  }
  if (zmq_connect(heartbeat_socket_, buildAddr(PORT_CONTROL).c_str()) != 0) {
    std::cerr << "[NetworkReceiver] Failed to connect heartbeat socket\n";
    return false;
  }

  std::cout << "[NetworkReceiver] Connected to all Pi streams\n";

  // Measure initial RTT for latency correction on ZMQ-received timestamps
  double rtt_ms = measureRTT(pi_ip_);
  if (rtt_ms > 0) {
    uint64_t one_way_ns = static_cast<uint64_t>(rtt_ms * 0.5 * 1e6);
    one_way_latency_ns_.store(one_way_ns, std::memory_order_relaxed);
    std::cout << "[NetworkReceiver] RTT=" << rtt_ms
              << "ms, one-way latency=" << (rtt_ms / 2.0) << "ms ("
              << one_way_ns << "ns)\n";
  } else {
    std::cout << "[NetworkReceiver] WARNING: Could not measure RTT, "
              << "timestamps will not be latency-corrected\n";
  }

  running_.store(true);

  // Start receiver threads
  cam_thread_ = std::thread(&NetworkReceiver::cameraThread, this);
  radar_l_thread_ = std::thread(&NetworkReceiver::radarLThread, this);
  radar_r_thread_ = std::thread(&NetworkReceiver::radarRThread, this);
  imu_thread_ = std::thread(&NetworkReceiver::imuThread, this);
  heartbeat_thread_ = std::thread(&NetworkReceiver::heartbeatThread, this);

  return true;
}

void NetworkReceiver::stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  // Wait for threads
  if (cam_thread_.joinable())
    cam_thread_.join();
  if (radar_l_thread_.joinable())
    radar_l_thread_.join();
  if (radar_r_thread_.joinable())
    radar_r_thread_.join();
  if (imu_thread_.joinable())
    imu_thread_.join();
  if (heartbeat_thread_.joinable())
    heartbeat_thread_.join();

  // Close sockets
  if (cam_socket_) {
    zmq_close(cam_socket_);
    cam_socket_ = nullptr;
  }
  if (radar_l_socket_) {
    zmq_close(radar_l_socket_);
    radar_l_socket_ = nullptr;
  }
  if (radar_r_socket_) {
    zmq_close(radar_r_socket_);
    radar_r_socket_ = nullptr;
  }
  if (imu_socket_) {
    zmq_close(imu_socket_);
    imu_socket_ = nullptr;
  }
  if (heartbeat_socket_) {
    zmq_close(heartbeat_socket_);
    heartbeat_socket_ = nullptr;
  }

  std::cout << "[NetworkReceiver] Stopped\n";
}

void NetworkReceiver::cameraThread() {
  std::vector<uint8_t> buffer(4096);

  while (running_.load()) {
    int len = zmq_recv(cam_socket_, buffer.data(), buffer.size(), 0);
    if (len < 0) {
      continue; // Timeout or error
    }

    if (static_cast<size_t>(len) < sizeof(PiMessageHeader)) {
      stats_.errors++;
      continue;
    }

    // Parse header
    PiMessageHeader header;
    std::memcpy(&header, buffer.data(), sizeof(header));

    if (!validateHeader(header) || static_cast<MessageType>(header.msg_type) !=
                                       MessageType::REAR_CAM_FRAME) {
      stats_.errors++;
      continue;
    }

    // Check for drops
    if (stats_.cam_frames > 0 && header.sequence != last_cam_seq_ + 1) {
      stats_.drops += header.sequence - last_cam_seq_ - 1;
    }
    last_cam_seq_ = header.sequence;

#pragma pack(                                                                  \
    push, 1) // Remove padding between floats and ints to match python struct
    struct RCWPayload {
      float ttc;
      float distance;
      int class_id;
    };
#pragma pack(pop)

    if (header.payload_size < sizeof(RCWPayload)) {
      stats_.errors++;
      continue;
    }

    // Parse rcw payload
    RCWPayload rcw_payload;
    std::memcpy(&rcw_payload, buffer.data() + sizeof(PiMessageHeader),
                sizeof(RCWPayload));

    // Create alert and push to queue
    if (cam_queue_) {
      Alert alert = rearCamConvert(rcw_payload.ttc, rcw_payload.distance,
                                   rcw_payload.class_id);
      cam_queue_->try_push(std::move(alert));
    }

    stats_.cam_frames++;
  }
}

void NetworkReceiver::radarLThread() {
  std::vector<uint8_t> buffer(4096);

  while (running_.load()) {
    int len = zmq_recv(radar_l_socket_, buffer.data(), buffer.size(), 0);
    if (len < 0)
      continue;

    if (static_cast<size_t>(len) < sizeof(PiMessageHeader)) {
      stats_.errors++;
      continue;
    }

    PiMessageHeader header;
    std::memcpy(&header, buffer.data(), sizeof(header));

    if (!validateHeader(header) || static_cast<MessageType>(header.msg_type) !=
                                       MessageType::REAR_RADAR_L) {
      stats_.errors++;
      continue;
    }

    // Check drops
    if (stats_.radar_l_packets > 0 &&
        header.sequence != last_radar_l_seq_ + 1) {
      stats_.drops += header.sequence - last_radar_l_seq_ - 1;
    }
    last_radar_l_seq_ = header.sequence;

    // Parse radar payload
    // Using simple format: [presence | range_cm (2 bytes)] from
    // RadarPayloadHeader + raw bytes
    size_t payload_size = header.payload_size;
    if (payload_size >= sizeof(RadarPayloadHeader) + 3) {
      RadarPayloadHeader rad_header;
      std::memcpy(&rad_header, buffer.data() + sizeof(PiMessageHeader),
                  sizeof(rad_header));

      const uint8_t *raw_data =
          buffer.data() + sizeof(PiMessageHeader) + sizeof(RadarPayloadHeader);

      uint8_t presence = raw_data[0];
      uint16_t range_cm = raw_data[1] | (raw_data[2] << 8);

      if (presence > 0 && range_cm > 0 && radar_l_queue_) {
        const uint64_t jetson_arrival_ns = Clock::now_ns();
        RadarTargets targets;
        targets.h.mount = Mount::RearCornerRadarL;
        targets.h.seq = header.sequence;
        targets.h.t_device_ns = header.timestamp_ns;
        targets.h.t_ingest_ns = jetson_arrival_ns;
        targets.h.healthy = true;

        RadarTarget target;
        target.range_m = static_cast<float>(range_cm) / 100.0f;
        target.azimuth_rad = 0.0f;
        target.radial_vel_mps = 0.0f;
        target.rcs_db = 0.0f;
        target.sigma_r = 0.1f;
        target.sigma_az = 0.5f;
        target.sigma_v = 1.0f;

        targets.targets.push_back(target);

        if (auto *rec = recorder_.load(std::memory_order_acquire)) {
          rec->recordRadar(targets);
        }

        radar_l_queue_->try_push(std::move(targets));
      }
    }

    stats_.radar_l_packets++;
  }
}

void NetworkReceiver::radarRThread() {
  std::vector<uint8_t> buffer(4096);

  while (running_.load()) {
    int len = zmq_recv(radar_r_socket_, buffer.data(), buffer.size(), 0);
    if (len < 0)
      continue;

    if (static_cast<size_t>(len) < sizeof(PiMessageHeader)) {
      stats_.errors++;
      continue;
    }

    PiMessageHeader header;
    std::memcpy(&header, buffer.data(), sizeof(header));

    if (!validateHeader(header) || static_cast<MessageType>(header.msg_type) !=
                                       MessageType::REAR_RADAR_R) {
      stats_.errors++;
      continue;
    }

    if (stats_.radar_r_packets > 0 &&
        header.sequence != last_radar_r_seq_ + 1) {
      stats_.drops += header.sequence - last_radar_r_seq_ - 1;
    }
    last_radar_r_seq_ = header.sequence;

    // Parse radar payload
    size_t payload_size = header.payload_size;
    if (payload_size >= sizeof(RadarPayloadHeader) + 3) {
      RadarPayloadHeader rad_header;
      std::memcpy(&rad_header, buffer.data() + sizeof(PiMessageHeader),
                  sizeof(rad_header));

      const uint8_t *raw_data =
          buffer.data() + sizeof(PiMessageHeader) + sizeof(RadarPayloadHeader);

      uint8_t presence = raw_data[0];
      uint16_t range_cm = raw_data[1] | (raw_data[2] << 8);

      if (presence > 0 && range_cm > 0 && radar_r_queue_) {
        const uint64_t jetson_arrival_ns = Clock::now_ns();
        RadarTargets targets;
        targets.h.mount = Mount::RearCornerRadarR;
        targets.h.seq = header.sequence;
        targets.h.t_device_ns = header.timestamp_ns;
        targets.h.t_ingest_ns = jetson_arrival_ns;
        targets.h.healthy = true;

        RadarTarget target;
        target.range_m = static_cast<float>(range_cm) / 100.0f;
        target.azimuth_rad = 0.0f;
        target.radial_vel_mps = 0.0f;
        target.rcs_db = 0.0f;
        target.sigma_r = 0.1f;
        target.sigma_az = 0.5f;
        target.sigma_v = 1.0f;

        targets.targets.push_back(target);

        if (auto *rec = recorder_.load(std::memory_order_acquire)) {
          rec->recordRadar(targets);
        }

        radar_r_queue_->try_push(std::move(targets));
      }
    }

    stats_.radar_r_packets++;
  }
}

void NetworkReceiver::imuThread() {
  std::vector<uint8_t> buffer(256);

  // Debug: track samples received in last 5 seconds
  auto last_debug_time = std::chrono::steady_clock::now();
  uint64_t samples_in_window = 0;
  uint64_t errors_in_window = 0;
  float last_accel_z = 0.0f;

  while (running_.load()) {
    int len = zmq_recv(imu_socket_, buffer.data(), buffer.size(), 0);
    if (len < 0) {
      // Check debug timer even on timeout (only if verbose mode enabled)
      if (g_verbose_mode.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           now - last_debug_time)
                           .count();
        if (elapsed >= 5) {
          std::cout << "\n[IMU DEBUG] Last 5s: received=" << samples_in_window
                    << ", errors=" << errors_in_window
                    << ", total_samples=" << stats_.imu_samples
                    << ", total_errors=" << stats_.errors
                    << ", last_accel_z=" << last_accel_z << "\n"
                    << std::flush;
          samples_in_window = 0;
          errors_in_window = 0;
          last_debug_time = now;
        }
      }
      continue;
    }

    if (static_cast<size_t>(len) <
        sizeof(PiMessageHeader) + sizeof(ImuPayload)) {
      stats_.errors++;
      errors_in_window++;
      continue;
    }

    PiMessageHeader header;
    std::memcpy(&header, buffer.data(), sizeof(header));

    if (!validateHeader(header) ||
        static_cast<MessageType>(header.msg_type) != MessageType::IMU_SAMPLE) {
      stats_.errors++;
      errors_in_window++;
      continue;
    }

    if (stats_.imu_samples > 0 && header.sequence != last_imu_seq_ + 1) {
      stats_.drops += header.sequence - last_imu_seq_ - 1;
    }
    last_imu_seq_ = header.sequence;

    // Parse IMU payload
    ImuPayload imu_payload;
    std::memcpy(&imu_payload, buffer.data() + sizeof(PiMessageHeader),
                sizeof(imu_payload));

    // Create ImuSample and push to queue
    if (imu_queue_) {
      // Use Jetson's clock at arrival time as the authoritative timestamp.
      // sample.t_capture (Pi clock corrected for latency) is kept for data
      // integrity but MUST NOT be used for recording — the Pi's wall clock
      // may be unsynchronised from the Jetson's CLOCK_MONOTONIC_RAW.
      const uint64_t jetson_arrival_ns = Clock::now_ns();

      ImuSample sample;
      sample.t_capture = jetson_arrival_ns;
      sample.accel = {imu_payload.accel_x, imu_payload.accel_y,
                      imu_payload.accel_z};
      sample.gyro = {imu_payload.gyro_x, imu_payload.gyro_y,
                     imu_payload.gyro_z};
      sample.mag = {imu_payload.mag_x, imu_payload.mag_y, imu_payload.mag_z};
      sample.quat = {imu_payload.quat_w, imu_payload.quat_x, imu_payload.quat_y,
                     imu_payload.quat_z};
      sample.temperature = imu_payload.temperature;
      sample.calibration_status = imu_payload.calibration_status;

      // Record with Jetson arrival time
      if (auto *rec = recorder_.load(std::memory_order_acquire)) {
        rec->recordIMU(sample);
      }

      imu_queue_->try_push(std::move(sample));
      last_accel_z = imu_payload.accel_z;
    }

    stats_.imu_samples++;
    samples_in_window++;

    // Debug output every 5 seconds (only if verbose mode enabled)
    if (g_verbose_mode.load()) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         now - last_debug_time)
                         .count();
      if (elapsed >= 5) {
        std::cout << "\n[IMU DEBUG] Last 5s: received=" << samples_in_window
                  << ", errors=" << errors_in_window
                  << ", total_samples=" << stats_.imu_samples
                  << ", total_errors=" << stats_.errors
                  << ", last_accel_z=" << last_accel_z << "\n"
                  << std::flush;
        samples_in_window = 0;
        errors_in_window = 0;
        last_debug_time = now;
      }
    }
  }
}

void NetworkReceiver::heartbeatThread() {
  std::vector<uint8_t> buffer(256);
  auto last_heartbeat = std::chrono::steady_clock::now();

  while (running_.load()) {
    // Build and send heartbeat request (REQ/REP pattern)
    PiMessageHeader req_header;
    req_header.magic = PI_MAGIC;
    req_header.version = PI_PROTOCOL_VERSION;
    req_header.msg_type = static_cast<uint16_t>(MessageType::HEARTBEAT);
    req_header.payload_size = 0;
    req_header._padding = 0;
    req_header.timestamp_ns = 0;
    req_header.sequence = 0;
    req_header.reserved = 0;

    if (zmq_send(heartbeat_socket_, &req_header, sizeof(req_header), 0) < 0) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    // Receive response
    int len = zmq_recv(heartbeat_socket_, buffer.data(), buffer.size(), 0);
    if (len >=
        static_cast<int>(sizeof(PiMessageHeader) + sizeof(HeartbeatPayload))) {
      PiMessageHeader header;
      std::memcpy(&header, buffer.data(), sizeof(header));

      if (validateHeader(header) &&
          static_cast<MessageType>(header.msg_type) == MessageType::HEARTBEAT) {

        HeartbeatPayload hb;
        std::memcpy(&hb, buffer.data() + sizeof(PiMessageHeader), sizeof(hb));

        stats_.pi_connected = true;
        stats_.heartbeats++;
        stats_.last_chrony_offset_us = hb.chrony_offset_us;
        last_heartbeat = std::chrono::steady_clock::now();

        // Warn if clock drift is too high
        if (std::abs(hb.chrony_offset_us) > 5000) { // > 5ms
          std::cerr << "[NetworkReceiver] WARNING: Chrony offset "
                    << hb.chrony_offset_us << "us - check time sync!\n";
        }
      }
    }

    // Check for heartbeat timeout
    auto elapsed = std::chrono::steady_clock::now() - last_heartbeat;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 5) {
      if (stats_.pi_connected) {
        std::cerr << "[NetworkReceiver] Pi heartbeat timeout - disconnected\n";
        stats_.pi_connected = false;
      }
    }

    // Wait 1 second between heartbeats
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// Static discovery function
std::vector<NetworkReceiver::DiscoveredDevice>
NetworkReceiver::discoverDevices(const std::string &pi_ip, int timeout_ms) {
  std::vector<DiscoveredDevice> devices;

  void *ctx = zmq_ctx_new();
  if (!ctx)
    return devices;

  void *sock = zmq_socket(ctx, ZMQ_REQ);
  if (!sock) {
    zmq_ctx_term(ctx);
    return devices;
  }

  // Set timeout
  zmq_setsockopt(sock, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
  zmq_setsockopt(sock, ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));

  std::string addr = "tcp://" + pi_ip + ":" + std::to_string(PORT_CONTROL);
  if (zmq_connect(sock, addr.c_str()) != 0) {
    std::cerr << "[Discovery] Failed to connect to " << addr << "\n";
    zmq_close(sock);
    zmq_ctx_term(ctx);
    return devices;
  }

  // Send discovery request
  PiMessageHeader req_header;
  req_header.magic = PI_MAGIC;
  req_header.version = PI_PROTOCOL_VERSION;
  req_header.msg_type = static_cast<uint16_t>(MessageType::DISCOVERY_REQ);
  req_header.payload_size = 0;
  req_header.timestamp_ns = 0;
  req_header.sequence = 0;
  req_header.reserved = 0;

  if (zmq_send(sock, &req_header, sizeof(req_header), 0) < 0) {
    std::cerr << "[Discovery] Failed to send request\n";
    zmq_close(sock);
    zmq_ctx_term(ctx);
    return devices;
  }

  // Receive response
  std::vector<uint8_t> buffer(1024);
  int len = zmq_recv(sock, buffer.data(), buffer.size(), 0);

  if (len < static_cast<int>(sizeof(PiMessageHeader) +
                             sizeof(DiscoveryResponsePayload))) {
    std::cerr << "[Discovery] No response or invalid response\n";
    zmq_close(sock);
    zmq_ctx_term(ctx);
    return devices;
  }

  // Parse response
  PiMessageHeader resp_header;
  std::memcpy(&resp_header, buffer.data(), sizeof(resp_header));

  if (!validateHeader(resp_header) ||
      static_cast<MessageType>(resp_header.msg_type) !=
          MessageType::DISCOVERY_RSP) {
    std::cerr << "[Discovery] Invalid response header\n";
    zmq_close(sock);
    zmq_ctx_term(ctx);
    return devices;
  }

  DiscoveryResponsePayload resp;
  std::memcpy(&resp, buffer.data() + sizeof(PiMessageHeader), sizeof(resp));

  // Parse device list
  size_t offset = sizeof(PiMessageHeader) + sizeof(DiscoveryResponsePayload);
  for (int i = 0; i < resp.num_devices; i++) {
    if (offset + sizeof(DeviceInfo) > static_cast<size_t>(len))
      break;

    DeviceInfo info;
    std::memcpy(&info, buffer.data() + offset, sizeof(info));
    offset += sizeof(DeviceInfo);

    DiscoveredDevice dev;
    dev.type = static_cast<DeviceType>(info.device_type);
    dev.mount = static_cast<MountId>(info.mount_id);
    dev.status = static_cast<DeviceStatus>(info.status);
    dev.serial =
        std::string(info.serial, strnlen(info.serial, sizeof(info.serial)));
    devices.push_back(dev);
  }

  zmq_close(sock);
  zmq_ctx_term(ctx);

  return devices;
}

// Static RTT measurement using ZMQ
double NetworkReceiver::measureRTT(const std::string &pi_ip) {
  void *ctx = zmq_ctx_new();
  if (!ctx)
    return -1.0;

  void *sock = zmq_socket(ctx, ZMQ_REQ);
  if (!sock) {
    zmq_ctx_term(ctx);
    return -1.0;
  }

  int timeout = 3000;
  zmq_setsockopt(sock, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
  zmq_setsockopt(sock, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));

  std::string addr = "tcp://" + pi_ip + ":" + std::to_string(PORT_CONTROL);
  if (zmq_connect(sock, addr.c_str()) != 0) {
    zmq_close(sock);
    zmq_ctx_term(ctx);
    return -1.0;
  }

  // Create a simple ping message (discovery request works as ping)
  PiMessageHeader ping;
  ping.magic = PI_MAGIC;
  ping.version = PI_PROTOCOL_VERSION;
  ping.msg_type = static_cast<uint16_t>(MessageType::DISCOVERY_REQ);
  ping.payload_size = 0;
  ping.timestamp_ns = 0;
  ping.sequence = 0;
  ping.reserved = 0;

  auto start = std::chrono::high_resolution_clock::now();

  if (zmq_send(sock, &ping, sizeof(ping), 0) < 0) {
    zmq_close(sock);
    zmq_ctx_term(ctx);
    return -1.0;
  }

  std::vector<uint8_t> buffer(1024);
  int len = zmq_recv(sock, buffer.data(), buffer.size(), 0);

  auto end = std::chrono::high_resolution_clock::now();

  zmq_close(sock);
  zmq_ctx_term(ctx);

  if (len < 0) {
    return -1.0;
  }

  return std::chrono::duration<double, std::milli>(end - start).count();
}

// Converts rcw payload to alert
static Alert rearCamConvert(float ttc_s, float range_m, int object_class) {
  Alert alert;
  float timestamp_ns = Clock::now_ns();

  // Timestamp in milliseconds
  alert.t_ms = timestamp_ns / 1'000'000; // TODO set outside

  // Unique ID (tick based, no track_id in FCWAlert)
  std::ostringstream id;
  id << "rcw-" << (timestamp_ns / 50'000'000) << "-" << object_class;
  alert.id = id.str();

  alert.type = AlertType::RCW;

  // Direction is front
  alert.direction = "rear";

  // Severity based on TTC
  if (ttc_s < 1.0f) {
    alert.severity = Severity::Critical;
  } else if (ttc_s < 2.0f) {
    alert.severity = Severity::Warning;
  } else {
    alert.severity = Severity::Info;
  }

  // TTL: 1 second
  alert.ttl_ms = 1000;

  // Rationale as JSON
  std::ostringstream rationale;
  rationale << std::fixed << std::setprecision(2);
  rationale << "{\"ttc_s\":" << ttc_s << ",\"range_m\":" << range_m
            << ",\"class\":\"" << object_class << "\"}";
  alert.rationale = rationale.str();

  // Object ID (use object_class since track_id not available)
  alert.object_id = object_class;

  // Sources
  alert.sources = {"RearCam"};

  // Schema version
  alert.schemaVersion = "v1.0";

  // Confidence (default) //TODO can add conf to tracks
  alert.confidence = 0.9f;

  return alert;
}

} // namespace adas
