// File: src/main.cpp
// ADAS Pipeline Entry Point - Stage A (Ingest & Timestamp)
#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/stage_a/DeviceWizard.hpp"
#include "adas/stage_a/IngestManager.hpp"
#include "adas/stage_e/EgoFrame.hpp"
#include "adas/stage_e/Track.hpp"

#include <csignal>
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <thread>

namespace {
adas::SPSCQueue<adas::RadarTargets, 8> queue_;
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
    std::cout << "Parsed target - Range: " << range << " m, Speed: " << speed << " m/s" << std::endl;
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

    std::cout << "[RadarIngest] Starting on reading from file" << std::endl;

    std::getline(file, line); //skip header line


    while (std::getline(file, line)) {
        // Timestamp immediately after successful read
        uint64_t t_ingest = adas::Clock::now_ns();
        std::string line2;
        std::getline(file, line2);
        // Parse and push to queue
        adas::RadarTargets targets = parseFrame(line+line2, t_ingest);
        queue_.try_push(std::move(targets));

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void setupEgoFrame(){
	float ef_x = 0;
	float ef_y = 0;
	float ef_vx = 0;
	float ef_vy = 0;
	float ef_yaw = 0;

    cv::Mat egoFrameInitialState = (cv::Mat_<float>(5,1) << ef_x, ef_y, ef_vx, ef_vy, ef_yaw); //Vehicle at 0,0 moving 5x,0y with 0 yaw
    adas::EgoFrame egoFrame = adas::EgoFrame(egoFrameInitialState);
}

void getRadarTargets(){
    std::optional<adas::RadarTargets> radarTargets;
    adas::Track track = adas::Track();

    while(true){
        if (auto radarTargets = queue_.try_pop()) {    
            if(radarTargets.has_value()){
                //Process radar targets
                float sum_speed = 0.0f;
                float avg_speed = 0.0f;
                float final_range = 0.0f;
                for (adas::RadarTarget target : radarTargets->targets){
                    //Process each target
                    sum_speed += target.radial_vel_mps;
                }

                avg_speed = sum_speed / radarTargets->targets.size();
                final_range = radarTargets->targets.back().range_m;
                std::cout << "Avg Speed: " << avg_speed << " m/s, Final Range: " << final_range << " m" << std::endl;
                if(avg_speed <= 0.0 && avg_speed >= -0.02 && final_range <= 2.42 && final_range >= 2.38){//TODO get rid of 2.4
                    track = adas::Track();
                }else if(!track.kf_initialized){
                    std::cout << "Initializing Track" << std::endl;
                    track = adas::Track((cv::Mat_<float>(4,1) << final_range, 0.0f, avg_speed, 0.0f));
                }else{
                    std::cout << "Update Track" << std::endl;
                    track = track.update((cv::Mat_<float>(4,1) << final_range, 0.0f, avg_speed, 0.0f),radarTargets->h.t_ingest_ns);
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
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

    std::thread t1(readFromFile);
    std::thread t2(getRadarTargets);

    while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    t1.detach();
    t2.detach();

    t1.join();
    t2.join();
}
