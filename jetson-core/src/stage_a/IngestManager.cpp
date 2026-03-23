// File: src/stage_a/IngestManager.cpp
// Lifecycle manager implementation for Stage A
#include "adas/stage_a/IngestManager.hpp"
#include "adas/stage_a/DeviceWizard.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace adas {

IngestManager::IngestManager(const Config &config, const HardwareMap &hw_map)
    : config_(config), hw_map_(hw_map), is_replay_mode_(false) {}

IngestManager::~IngestManager() { stop(); }

bool IngestManager::initReplay(const std::string &file_path, float speed) {
  if (running_.load(std::memory_order_relaxed)) {
    std::cerr << "[IngestManager] Cannot init replay while running\n";
    return false;
  }

  std::cout << "[IngestManager] Initializing Replay Mode with file: "
            << file_path << "\n";
  replay_engine_ = std::make_unique<ReplayEngine>();

  if (!replay_engine_->load(file_path)) {
    std::cerr << "[IngestManager] Failed to load replay file\n";
    replay_engine_.reset();
    return false;
  }

  // Set speed
  replay_engine_->setSpeed(speed);

  // Wire queues
  replay_engine_->setFrontDetQueue(&det_front_ds_queue_);
  replay_engine_->setRcwQueue(&rcw_queue_);
  replay_engine_->setRadarQueue(Mount::FrontRadar, &radar_front_queue_);
  replay_engine_->setRadarQueue(Mount::RearCornerRadarL, &radar_rear_l_queue_);
  replay_engine_->setRadarQueue(Mount::RearCornerRadarR, &radar_rear_r_queue_);
  replay_engine_->setIMUQueue(&imu_queue_);

  is_replay_mode_ = true;
  return true;
}

void IngestManager::start() {
  if (running_.load(std::memory_order_relaxed)) {
    return;
  }

  std::cout << "\n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "               STAGE A: INGEST & TIMESTAMP                    \n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "  Starting all sensor ingest threads...                       \n";
  std::cout
      << "==============================================================\n";
  std::cout << "\n";

  running_.store(true, std::memory_order_relaxed);

  if (is_replay_mode_ && replay_engine_) {
    std::cout << "[IngestManager] Replay Engine taking over Ingest Layer\n";
    replay_engine_->start();
  } else {
    // Launch in order of priority (LIVE MODE)
    std::cout << "[IngestManager] FrontCam: managed by external DeepStream\n";
    launchFrontRadar();
    launchPiReceiver();
  }

  std::cout << "\n[IngestManager] All ingest threads started\n";
}

void IngestManager::stop() {
  if (!running_.load(std::memory_order_relaxed)) {
    return;
  }

  std::cout << "\n[IngestManager] Stopping all ingest threads...\n";
  running_.store(false, std::memory_order_relaxed);

#ifdef HAS_ZMQ
  if (ds_receiver_) {
    ds_receiver_->stop();
  }
  if (zmq_receiver_) {
    zmq_receiver_->stop();
  }
#endif
  if (radar_front_) {
    radar_front_->stop();
  }

  if (replay_engine_) {
    replay_engine_->stop();
  }

  std::cout << "[IngestManager] All threads stopped (FR93 graceful shutdown)\n";
}

void IngestManager::launchPiReceiver() {
#ifdef HAS_ZMQ
  auto extractPiIp = [this](Mount mount) -> std::string {
    auto it = hw_map_.mappings.find(mount);
    if (it == hw_map_.mappings.end()) {
      return "";
    }
    const std::string &addr = it->second;
    if (addr.rfind("zmq://", 0) != 0) {
      return "";
    }
    const size_t colon = addr.find(':', 6);
    if (colon == std::string::npos || colon <= 6) {
      return "";
    }
    return addr.substr(6, colon - 6);
  };

  std::string pi_ip = extractPiIp(Mount::RearCornerRadarL);
  if (pi_ip.empty()) {
    pi_ip = extractPiIp(Mount::RearCornerRadarR);
  }
  if (pi_ip.empty()) {
    pi_ip = extractPiIp(Mount::IMU);
  }
  if (pi_ip.empty()) {
    std::cout << "[IngestManager] No Pi radar/IMU mapping found, skipping Pi "
                 "ZMQ receiver\n";
  } else {
    pi_ip_ = pi_ip; // cache for getPiIp()

    std::cout << "[IngestManager] Launching ZMQ receiver for Pi at " << pi_ip
              << "...\n";

    // Create NetworkReceiver (constructor takes only IP)
    zmq_receiver_ = std::make_unique<NetworkReceiver>(pi_ip);

    // Start with queue pointers
    if (zmq_receiver_->start(&rcw_queue_, &imu_queue_, &radar_rear_l_queue_,
                             &radar_rear_r_queue_)) {
      std::cout << "[IngestManager] ZMQ receiver started for Pi RCW/radar/IMU "
                   "streams\n";
    } else {
      std::cerr << "[IngestManager] Failed to start ZMQ receiver\n";
      zmq_receiver_.reset();
    }
  }

  // Launch local IPC receiver for DeepStream
  ds_receiver_ = std::make_unique<DeepStreamReceiver>();
  if (ds_receiver_->start(&det_front_ds_queue_)) {
    std::cout
        << "[IngestManager] DeepStream IPC receiver started for Front Camera\n";
  } else {
    std::cerr << "[IngestManager] Failed to start DeepStream IPC receiver\n";
    ds_receiver_.reset();
  }
#endif
}

void IngestManager::launchFrontRadar() {
  std::cout << "[IngestManager] Launching front radar ingest...\n";

  radar_front_ =
      std::make_unique<RadarIngest>(Mount::FrontRadar, config_.front_radar.port,
                                    radar_front_queue_, config_.front_radar);
  radar_front_->start();
}

SPSCQueue<RadarTargets, 8> &IngestManager::getRadarQueue(Mount mount) {
  switch (mount) {
  case Mount::FrontRadar:
    return radar_front_queue_;
  case Mount::RearCornerRadarL:
    return radar_rear_l_queue_;
  case Mount::RearCornerRadarR:
    return radar_rear_r_queue_;
  default:
    throw std::out_of_range("Invalid radar mount");
  }
}

IngestManager::HealthStatus IngestManager::getHealth() const {
  HealthStatus status;
  // Read from Replay Engine if active
  if (is_replay_mode_ && replay_engine_) {
    status.all_healthy = true;
    status.total_drops = 0;

    // Build summary for Replay Mode
    std::ostringstream ss;
    if (replay_engine_->isFinished()) {
      ss << "REPLAY FINISHED (100%)";
    } else {
      ss << "REPLAY RUNNING (" << std::fixed << std::setprecision(1)
         << (replay_engine_->getProgress() * 100.0f) << "%" << ")";
    }
    status.summary = ss.str();
    return status;
  }

  // FrontCam health is managed by external DeepStream.
  // We still report queue drop count to surface backpressure issues.
  status.sensor_health[Mount::FrontCam] =
      true; // assume OK; DeepStream process logs its own health
  status.total_drops += det_front_ds_queue_.drops();
  status.total_drops += rcw_queue_.drops();

  // Check Pi-fed rear sector streams (RCW, rear radars, IMU)
#ifdef HAS_ZMQ
  if (zmq_receiver_) {
    bool h = zmq_receiver_->isPiConnected();
    status.sensor_health[Mount::IMU] = h; // IMU also comes from Pi
    status.sensor_health[Mount::RearCornerRadarL] = h;
    status.sensor_health[Mount::RearCornerRadarR] = h;
    status.total_drops += imu_queue_.drops();
    status.total_drops += radar_rear_l_queue_.drops();
    status.total_drops += radar_rear_r_queue_.drops();
    if (!h) {
      status.all_healthy = false;
    }
  }
#endif

  if (!pi_ip_.empty() && !status.sensor_health.count(Mount::IMU)) {
    status.sensor_health[Mount::IMU] = false;
    status.sensor_health[Mount::RearCornerRadarL] = false;
    status.sensor_health[Mount::RearCornerRadarR] = false;
    status.all_healthy = false;
  }

  // Check front radar
  if (radar_front_) {
    bool h = radar_front_->isHealthy();
    status.sensor_health[Mount::FrontRadar] = h;
    status.total_drops += radar_front_queue_.drops();
    if (!h) {
      status.all_healthy = false;
    }
  }

  // Build summary
  std::ostringstream ss;
  int healthy_count = 0;
  int total_count = 0;
  for (const auto &[mount, h] : status.sensor_health) {
    ++total_count;
    if (h) {
      ++healthy_count;
    }
  }
  ss << healthy_count << "/" << total_count << " sensors healthy";
  if (status.total_drops > 0) {
    ss << ", " << status.total_drops << " drops";
  }
  status.summary = ss.str();

  return status;
}

void IngestManager::printStatus() const {
  auto status = getHealth();

  std::cout << "\n";
  std::cout
      << "----------------------------------------------------------------\n";
  std::cout
      << "                    STAGE A STATUS                              \n";
  std::cout
      << "----------------------------------------------------------------\n";

  for (const auto &[mount, healthy] : status.sensor_health) {
    std::cout << "  " << std::left << std::setw(20) << mountToString(mount)
              << (healthy ? "[OK] HEALTHY" : "[XX] UNHEALTHY") << "\n";
  }

  std::cout
      << "----------------------------------------------------------------\n";
  std::cout << "  " << std::left << std::setw(45) << status.summary << "\n";
  std::cout
      << "----------------------------------------------------------------\n";
}

void IngestManager::setRecorder(Recorder *recorder) {
  if (radar_front_)
    radar_front_->setRecorder(recorder);
#ifdef HAS_ZMQ
  if (zmq_receiver_)
    zmq_receiver_->setRecorder(recorder);
  if (ds_receiver_)
    ds_receiver_->setRecorder(recorder);
#endif
}

} // namespace adas
