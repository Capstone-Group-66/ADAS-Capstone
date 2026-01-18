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
bool stop = false;

adas::RadarTargets parseFrame(std::string str, uint64_t t_ingest) {
    adas::RadarTargets targets;

    // Build header
    targets.h = adas::Header(t_ingest, adas::Mount::FrontRadar, 0, true);

    //Look for speed
    size_t pos = 0;
    float speed = 0.0f;
    float range = 0.0f;
    while((pos = str.find("\"m", pos)) != std::string::npos){
        //Radar sends either "mps" for speed or "m" for range
        if(str[pos+2] == 'p'){
            pos += 6; // Skip "mps",

            // Extract number
            size_t num_start = pos;
            while (pos < str.size() &&
                (std::isdigit(str[pos]) || str[pos] == '.' ||
                    str[pos] == '-')) {
                ++pos;
            }

            if (pos > num_start) {
                speed = std::stof(str.substr(num_start, pos - num_start));
            }
        }else{
            pos += 4; // Skip "m",

            // Extract number
            size_t num_start = pos;
            while (pos < str.size() &&
                (std::isdigit(str[pos]) || str[pos] == '.' ||
                    str[pos] == '-')) {
                ++pos;
            }

            if (pos > num_start) {
                range = std::stof(str.substr(num_start, pos - num_start));
            }
        }

        adas::RadarTarget target;
        target.range_m = range;
        target.radial_vel_mps = speed;
        target.azimuth_rad = 0.0f; // OPS243-A doesn't provide azimuth
        target.rcs_db = 0.0f;
        target.sigma_r = 0.1f;
        target.sigma_v = 0.05f;
        target.sigma_az = 0.5f;
        targets.targets.push_back(target);
    }

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
    
    uint32_t previous_time_ms = adas::Clock::now_ms();
    std::string currentFrame = "";
    while (std::getline(file, line)) {
        // Timestamp immediately after successful read
        uint64_t t_ingest = adas::Clock::now_ns();
        // Parse and push to queue
        currentFrame += line;
        if(adas::Clock::ns_to_ms(t_ingest) - previous_time_ms >= 50){ //20Hz
            previous_time_ms = adas::Clock::ns_to_ms(t_ingest);
            adas::RadarTargets targets = parseFrame(currentFrame, t_ingest);
            FrontRadarQueue.try_push(std::move(targets));
            std::cout << "[RadarIngest] Pushed frame to FrontRadarQueue " << previous_time_ms << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    stop = true;
    std::cout << "[RadarIngest] Finished reading from file" << std::endl;
}
}

std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int signum) {
    std::cout << "\n[Main] Received signal " << signum << ", initiating shutdown...\n";
    g_shutdown_requested.store(true, std::memory_order_relaxed);
}

int main() {
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    adas::SensorFusion sensorFusion(FrontRadarQueue, IMUQueue);
    std::thread sensorFusionThread(&adas::SensorFusion::start, &sensorFusion);
    std::thread fileThread(readFromFile);

    while (!g_shutdown_requested.load(std::memory_order_relaxed) && !stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    fileThread.join();
    sensorFusion.stop();
}
