// File: src/stage_a/IngestManager.cpp
// Lifecycle manager implementation for Stage A
#include "adas/stage_a/IngestManager.hpp"
#include "adas/stage_a/DeviceWizard.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace adas {

IngestManager::IngestManager(const Config &config, const HardwareMap &hw_map)
    : config_(config), hw_map_(hw_map) {}

IngestManager::~IngestManager() { stop(); }

void IngestManager::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }

    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "               STAGE A: INGEST & TIMESTAMP                    \n";
    std::cout << "==============================================================\n";
    std::cout << "  Starting all sensor ingest threads...                       \n";
    std::cout << "==============================================================\n";
    std::cout << "\n";

    running_.store(true, std::memory_order_relaxed);

    // Launch in order of priority
    launchDirectCameras();
    launchFrontRadar();
    launchNetworkIngest();
    launchIMU(); // Will run in stub mode until hardware arrives

    std::cout << "\n[IngestManager] All ingest threads started\n";
}

void IngestManager::stop() {
    if (!running_.load(std::memory_order_relaxed)) {
        return;
    }

    std::cout << "\n[IngestManager] Stopping all ingest threads...\n";
    running_.store(false, std::memory_order_relaxed);

    // Stop in reverse order
    if (imu_) {
        imu_->stop();
    }
#ifdef HAS_ZMQ
    if (zmq_receiver_) {
        zmq_receiver_->stop();
    }
#endif
    if (network_) {
        network_->stop();
    }
    if (radar_front_) {
        radar_front_->stop();
    }
    if (cam_side_r_) {
        cam_side_r_->stop();
    }
    if (cam_side_l_) {
        cam_side_l_->stop();
    }
    if (cam_front_) {
        cam_front_->stop();
    }

    std::cout << "[IngestManager] All threads stopped (FR93 graceful shutdown)\n";
}

void IngestManager::launchDirectCameras() {
    std::cout << "[IngestManager] Launching direct USB cameras...\n";

    // FrontCam
    auto it = hw_map_.mappings.find(Mount::FrontCam);
    if (it != hw_map_.mappings.end()) {
        cam_front_ = std::make_unique<CameraIngest>(Mount::FrontCam, it->second, cam_front_queue_,
                                                    config_.cameras);
        cam_front_->start();
    } else {
        std::cerr << "[IngestManager] WARNING: FrontCam not in hardware_map\n";
    }

    // SideCamL
    it = hw_map_.mappings.find(Mount::SideCamL);
    if (it != hw_map_.mappings.end()) {
        cam_side_l_ = std::make_unique<CameraIngest>(Mount::SideCamL, it->second, cam_side_l_queue_,
                                                     config_.cameras);
        cam_side_l_->start();
    } else {
        std::cerr << "[IngestManager] WARNING: SideCamL not in hardware_map\n";
    }

    // SideCamR
    it = hw_map_.mappings.find(Mount::SideCamR);
    if (it != hw_map_.mappings.end()) {
        cam_side_r_ = std::make_unique<CameraIngest>(Mount::SideCamR, it->second, cam_side_r_queue_,
                                                     config_.cameras);
        cam_side_r_->start();
    } else {
        std::cerr << "[IngestManager] WARNING: SideCamR not in hardware_map\n";
    }
}

void IngestManager::launchNetworkIngest() {
#ifdef HAS_ZMQ
    // Check if Pi is configured in hardware map
    auto rear_cam_it = hw_map_.mappings.find(Mount::RearCam);
    if (rear_cam_it == hw_map_.mappings.end()) {
        std::cout << "[IngestManager] RearCam not in hardware_map, skipping ZMQ receiver\n";
        return;
    }

    // Extract Pi IP from hardware map (format: "zmq://IP:PORT")
    std::string addr = rear_cam_it->second;
    std::string pi_ip;
    if (addr.substr(0, 6) == "zmq://") {
        size_t colon = addr.find(':', 6);
        if (colon != std::string::npos) {
            pi_ip = addr.substr(6, colon - 6);
        }
    }

    if (pi_ip.empty()) {
        std::cerr << "[IngestManager] Invalid RearCam address: " << addr << "\n";
        std::cerr << "[IngestManager] Falling back to TCP NetworkIngest\n";
        network_ = std::make_unique<NetworkIngest>(cam_rear_queue_, radar_rear_l_queue_,
                                                   radar_rear_r_queue_, config_.network);
        network_->start();
        return;
    }

    std::cout << "[IngestManager] Launching ZMQ receiver for Pi at " << pi_ip << "...\n";

    // Create NetworkReceiver (constructor takes only IP)
    zmq_receiver_ = std::make_unique<NetworkReceiver>(pi_ip);

    // Start with queue pointers
    if (zmq_receiver_->start(&cam_rear_queue_, &imu_queue_)) {
        std::cout << "[IngestManager] ZMQ receiver started for RearCam + IMU\n";
    } else {
        std::cerr << "[IngestManager] Failed to start ZMQ receiver\n";
        zmq_receiver_.reset();
    }

#else
    std::cout << "[IngestManager] Launching TCP network ingest (Pi4 rear sector)...\n";

    network_ = std::make_unique<NetworkIngest>(cam_rear_queue_, radar_rear_l_queue_,
                                               radar_rear_r_queue_, config_.network);
    network_->start();
#endif
}

void IngestManager::launchFrontRadar() {
    std::cout << "[IngestManager] Launching front radar ingest...\n";

    radar_front_ = std::make_unique<RadarIngest>(Mount::FrontRadar, config_.front_radar.port,
                                                 radar_front_queue_, config_.front_radar);
    radar_front_->start();
}

void IngestManager::launchIMU() {
#ifdef HAS_ZMQ
    // If ZMQ receiver is active, Pi provides the IMU - skip local
    if (zmq_receiver_) {
        std::cout << "[IngestManager] Skipping local IMU (Pi provides IMU via ZMQ)\n";
        return;
    }
#endif
    std::cout << "[IngestManager] Launching IMU ingest (scaffold mode)...\n";

    imu_ = std::make_unique<IMUIngest>(imu_queue_, config_.imu);
    imu_->start();
}

SPSCQueue<CameraFrameData, 8> &IngestManager::getCameraQueue(Mount mount) {
    switch (mount) {
    case Mount::FrontCam:
        return cam_front_queue_;
    case Mount::SideCamL:
        return cam_side_l_queue_;
    case Mount::SideCamR:
        return cam_side_r_queue_;
    case Mount::RearCam:
        return cam_rear_queue_;
    default:
        throw std::out_of_range("Invalid camera mount");
    }
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
    status.all_healthy = true;
    status.total_drops = 0;

    // Check cameras
    if (cam_front_) {
        bool h = cam_front_->isHealthy();
        status.sensor_health[Mount::FrontCam] = h;
        status.total_drops += cam_front_queue_.drops();
        if (!h) {
            status.all_healthy = false;
        }
    }
    if (cam_side_l_) {
        bool h = cam_side_l_->isHealthy();
        status.sensor_health[Mount::SideCamL] = h;
        status.total_drops += cam_side_l_queue_.drops();
        if (!h) {
            status.all_healthy = false;
        }
    }
    if (cam_side_r_) {
        bool h = cam_side_r_->isHealthy();
        status.sensor_health[Mount::SideCamR] = h;
        status.total_drops += cam_side_r_queue_.drops();
        if (!h) {
            status.all_healthy = false;
        }
    }

    // Check network (rear sector)
#ifdef HAS_ZMQ
    if (zmq_receiver_) {
        bool h = zmq_receiver_->isPiConnected();
        status.sensor_health[Mount::RearCam] = h;
        status.sensor_health[Mount::IMU] = h; // IMU also comes from Pi
        status.total_drops += cam_rear_queue_.drops();
        status.total_drops += imu_queue_.drops();
        if (!h) {
            status.all_healthy = false;
        }
    } else
#endif
        if (network_) {
        bool h = network_->isHealthy();
        status.sensor_health[Mount::RearCam] = h;
        status.sensor_health[Mount::RearCornerRadarL] = h;
        status.sensor_health[Mount::RearCornerRadarR] = h;
        status.total_drops += cam_rear_queue_.drops();
        status.total_drops += radar_rear_l_queue_.drops();
        status.total_drops += radar_rear_r_queue_.drops();
        if (!h) {
            status.all_healthy = false;
        }
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

    // Check IMU (expected to be unhealthy until hardware arrives)
    if (imu_) {
        bool h = imu_->isHealthy();
        status.sensor_health[Mount::IMU] = h;
        // Don't count IMU against all_healthy since hardware is pending
    }

    // Build summary
    std::ostringstream ss;
    int healthy_count = 0;
    int total_count = 0;
    for (const auto &[mount, h] : status.sensor_health) {
        if (mount != Mount::IMU) { // Exclude IMU from count
            ++total_count;
            if (h) {
                ++healthy_count;
            }
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
    std::cout << "----------------------------------------------------------------\n";
    std::cout << "                    STAGE A STATUS                              \n";
    std::cout << "----------------------------------------------------------------\n";

    for (const auto &[mount, healthy] : status.sensor_health) {
        std::cout << "  " << std::left << std::setw(20) << mountToString(mount)
                  << (healthy ? "[OK] HEALTHY" : "[XX] UNHEALTHY") << "\n";
    }

    std::cout << "----------------------------------------------------------------\n";
    std::cout << "  " << std::left << std::setw(45) << status.summary << "\n";
    std::cout << "----------------------------------------------------------------\n";
}

} // namespace adas
