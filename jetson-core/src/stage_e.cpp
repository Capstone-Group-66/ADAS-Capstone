// File: src/main.cpp
// ADAS Pipeline Entry Point - Stage A (Ingest & Timestamp)
#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/stage_a/DeviceWizard.hpp"
#include "adas/stage_a/IngestManager.hpp"
#include "adas/stage_e/SensorFusion.hpp"

#include <csignal>
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <thread>

namespace {
adas::SPSCQueue<adas::RadarTargets, 8> FrontRadarQueue;
adas::SPSCQueue<adas::ImuSample, 32> IMUQueue;
std::thread thread_;

adas::RadarTargets parseFrame(std::string str, uint64_t t_ingest) {
    adas::RadarTargets targets;

    // Build header
    targets.h = adas::Header(t_ingest, adas::Mount::FrontRadar, 0, true);

    //Look for speed
    float speed = 0.0f;
    size_t speed_pos = 0;
    if ((speed_pos = str.find("mps\",", speed_pos)) != std::string::npos) {
        speed_pos += 5; // Skip "mps",":
        size_t speed_end = speed_pos;
        while (speed_end < str.size() &&
               (std::isdigit(str[speed_end]) || str[speed_end] == '.' ||
                str[speed_end] == '-')) {
            ++speed_end;
        }
        if (speed_end > speed_pos) {
            speed = std::stof(str.substr(speed_pos, speed_end - speed_pos));
        }
    } else {
        speed = 0.0f;
    }
    
    float range = 0.0f;
    float range_pos = 0;
    if ((range_pos = str.find("m\",", range_pos)) != std::string::npos) {
        range_pos += 3; // Skip "m",":
        size_t range_end = range_pos;
        while (range_end < str.size() &&
               (std::isdigit(str[range_end]) || str[range_end] == '.' ||
                str[range_end] == '-')) {
            ++range_end;
        }
        if (range_end > range_pos) {
            range = std::stof(str.substr(range_pos, range_end - range_pos));
        }
    } else {
        range = 0.0f;
    }
    
    adas::RadarTarget target;
    target.range_m = range;
    target.radial_vel_mps = speed;
    target.azimuth_rad = 0.0f; // OPS243-A doesn't provide azimuth
    target.rcs_db = 0.0f;
    target.sigma_r = 0.1f;
    target.sigma_v = 0.05f;
    target.sigma_az = 0.5f;
    //std::cout << "Parsed target - Range: " << range << " m, Speed: " << speed << " m/s" << std::endl;
    targets.targets.push_back(target);

    if (targets.targets.empty()) {
        // No targets detected - still valid, just empty
        targets.h.healthy = true;
    }

    return targets;
}

void readFromFile() {
    std::ifstream file("test_files/omnipresense_radar_log.csv");
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Cannot open file\n";
        return;
    }

    std::cout << "[RadarIngest] Start reading from file" << std::endl;

    std::getline(file, line); //skip header line


    while (std::getline(file, line)) {
        // Timestamp immediately after successful read
        uint64_t t_ingest = adas::Clock::now_ns();
        std::string line2;
        std::getline(file, line2);
        // Parse and push to queue
        adas::RadarTargets targets = parseFrame(line+line2, t_ingest);
        FrontRadarQueue.try_push(std::move(targets));

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
}

std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int signum) {
    std::cout << "\n[Main] Received signal " << signum << ", initiating shutdown...\n";
    g_shutdown_requested.store(true, std::memory_order_relaxed);
}

int main() {
    adas::SensorFusion sensorFusion(FrontRadarQueue, IMUQueue);
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // std::thread fileThread(readFromFile);
    std::thread sensorFusionThread(&adas::SensorFusion::start, &sensorFusion);

    while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // fileThread.join();
    sensorFusionThread.join();
}
