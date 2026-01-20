// File: src/main.cpp
// ADAS Pipeline Entry Point - Interactive CLI with Stages A, B
#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/common/Globals.hpp"
#include "adas/stage_a/DeviceWizard.hpp"
#include "adas/stage_a/IngestManager.hpp"
#include "adas/stage_b/CameraPipeline.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace {

// Global managers
std::unique_ptr<adas::IngestManager> g_ingest_manager;
std::unique_ptr<adas::StageBManager> g_stage_b_manager;
std::atomic<bool> g_shutdown_requested{false};
std::atomic<bool> g_pipeline_running{false};

// Status bar thread
std::thread g_status_thread;
std::atomic<bool> g_status_running{false};
std::chrono::steady_clock::time_point g_pipeline_start_time;

// Visualization thread
std::thread g_visualizer_thread;
std::atomic<bool> g_visualizer_running{false};

// Detection output queues (Stage B -> Stage E)
adas::SPSCQueue<adas::DetBatch, 8> g_det_front_queue;
adas::SPSCQueue<adas::DetBatch, 8> g_det_side_l_queue;
adas::SPSCQueue<adas::DetBatch, 8> g_det_side_r_queue;
adas::SPSCQueue<adas::DetBatch, 8> g_det_rear_queue;

std::string formatUptime(std::chrono::seconds uptime) {
    int hours = uptime.count() / 3600;
    int minutes = (uptime.count() % 3600) / 60;
    int seconds = uptime.count() % 60;
    
    std::ostringstream ss;
    if (hours > 0) {
        ss << hours << "h " << minutes << "m";
    } else if (minutes > 0) {
        ss << minutes << "m " << seconds << "s";
    } else {
        ss << seconds << "s";
    }
    return ss.str();
}

// Visualization thread
void visualizationThread() {
    std::cout << "[Visualizer] Thread started (waiting for detections...)\n";
    cv::namedWindow("Stage B: FrontCam", cv::WINDOW_AUTOSIZE);
    
    while (g_visualizer_running.load() && !g_shutdown_requested.load()) {
        adas::DetBatch batch;
        if (g_det_front_queue.try_pop(batch)) {
            // Check if we have a frame to visualize
            if (!batch.frame.empty()) {
                cv::Mat vis = batch.frame.clone();
                
                // Draw detections
                for (const auto& det : batch.dets) {
                    // Choose color based on class (simple hash)
                    cv::Scalar color(
                        (det.cls * 50) % 255, 
                        (det.cls * 80 + 100) % 255, 
                        (det.cls * 120 + 200) % 255
                    );
                    
                    // Draw box
                    cv::rectangle(vis, det.box_px, color, 2);
                    
                    // Draw label
                    std::string label = std::to_string(det.cls) + " " + 
                                        std::to_string(static_cast<int>(det.score * 100)) + "%";
                                        
                    int baseLine;
                    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    
                    cv::rectangle(vis, 
                        cv::Point(det.box_px.x, det.box_px.y - labelSize.height),
                        cv::Point(det.box_px.x + labelSize.width, det.box_px.y + baseLine),
                        color, cv::FILLED);
                        
                    cv::putText(vis, label, 
                        cv::Point(det.box_px.x, det.box_px.y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
                        
                    // Draw centroid
                    cv::circle(vis, det.centroid, 3, cv::Scalar(0, 255, 0), -1);
                }
                
                // Draw info
                std::string info = "Inference: " + std::to_string(batch.inference_time_us / 1000.0) + "ms";
                cv::putText(vis, info, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                cv::imshow("Stage B: FrontCam", vis);
                cv::waitKey(1);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
    cv::destroyWindow("Stage B: FrontCam");
    std::cout << "[Visualizer] Thread stopped\n";
}

void statusBarThread() {
    while (g_status_running.load() && !g_shutdown_requested.load()) {
        if (!g_ingest_manager) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        
        auto health = g_ingest_manager->getHealth();
        
        // Get individual sensor health
        bool front_cam_ok = false;
        bool front_radar_ok = false;
        bool imu_ok = false;
        
        auto& sh = health.sensor_health;
        if (sh.find(adas::Mount::FrontCam) != sh.end()) {
            front_cam_ok = sh[adas::Mount::FrontCam];
        }
        if (sh.find(adas::Mount::FrontRadar) != sh.end()) {
            front_radar_ok = sh[adas::Mount::FrontRadar];
        }
        if (sh.find(adas::Mount::IMU) != sh.end()) {
            imu_ok = sh[adas::Mount::IMU];
        }
        
        // Calculate uptime
        auto now = std::chrono::steady_clock::now();
        auto uptime = std::chrono::duration_cast<std::chrono::seconds>(now - g_pipeline_start_time);
        
        // Format status bar
        std::cout << "\r[ADAS] "
                  << "FrontCam:" << (front_cam_ok ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m") << " "
                  << "FrontRadar:" << (front_radar_ok ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m") << " "
                  << "IMU:" << (imu_ok ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m") << " | "
                  << "Drops:" << health.total_drops << " | "
                  << formatUptime(uptime) << "     " << std::flush;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "\n";  // Clean line after stopping
}

void signalHandler(int signum) {
    std::cout << "\n[Main] Received signal " << signum << ", initiating shutdown...\n";
    g_shutdown_requested.store(true, std::memory_order_relaxed);
}

void printBanner() {
    std::cout << R"(

    ===========================================================================
                                                                       
         AAAAA  DDDD    AAAAA  SSSSS      PPPP   III  PPPP   EEEEE     
        AA   AA DD  DD AA   AA SS        PP  PP  III PP  PP EE         
        AAAAAAA DD   DD AAAAAAA SSSSS    PPPPPP  III PPPPPP EEEEE      
        AA   AA DD  DD AA   AA     SS    PP      III PP     EE         
        AA   AA DDDD   AA   AA SSSSS     PP      III PP     EEEEE      
                                                                       
                    ADAS Pipeline - Interactive Mode                   
                                                                       
    ===========================================================================
    )" << std::endl;
    
    // Pause for 2 seconds so users can see the banner
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void printMenu() {
    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "                    MAIN MENU                                 \n";
    std::cout << "==============================================================\n";
    std::cout << "  1) Start Pipeline (Stages A + B)\n";
    std::cout << "  2) Stop Pipeline\n";
    std::cout << "  3) Show Status\n";
    std::cout << "  4) Run Device Wizard (USB cameras)\n";
    std::cout << "  5) Run Camera Calibration\n";
    std::cout << "  6) Register Pi4 Network Devices\n";
    std::cout << "  7) Test RTT to Pi4\n";
    std::cout << "  8) Toggle Verbose Mode [" << (adas::g_verbose_mode.load() ? "ON" : "OFF") << "]\n";
    std::cout << "  0) Exit\n";
    std::cout << "==============================================================\n";
    std::cout << "  Enter choice: ";
}

void startPipeline(const adas::Config& config, const adas::HardwareMap& hw_map,
                   const std::string& calib_dir, const std::string& model_path) {
    if (g_pipeline_running.load()) {
        std::cout << "[Main] Pipeline is already running\n";
        return;
    }
    
    std::cout << "\n[Main] Starting pipeline...\n";
    
    // Stage A: Ingest
    g_ingest_manager = std::make_unique<adas::IngestManager>(config, hw_map);
    g_ingest_manager->start();
    
    // Stage B: Camera Preprocessing + Inference
    g_stage_b_manager = std::make_unique<adas::StageBManager>(calib_dir, model_path);
    
    // Add camera pipelines for each mapped camera
    auto& mappings = hw_map.mappings;
    
    // FCW Vertical Slice: Only FrontCam is needed for Forward Collision Warning.
    // The FrontCam detections (DetBatch) will be fused with FrontRadar in Stage E.
    if (mappings.find(adas::Mount::FrontCam) != mappings.end()) {
        g_stage_b_manager->addCamera(adas::Mount::FrontCam,
                                     g_ingest_manager->getCameraQueue(adas::Mount::FrontCam),
                                     g_det_front_queue);
    }
    
    // TODO: Wire remaining cameras when implementing other alerts:
    // - SideCamL/R: Needed for Blind Spot Detection (BSD) and Lane Change Warning (LCW)
    // - RearCam: Needed for Rear Cross Traffic Alert (RCTA) - comes via NetworkIngest
    // These cameras do not contribute to FCW, so they're excluded from the vertical slice.
    /*
    if (mappings.find(adas::Mount::SideCamL) != mappings.end()) {
        g_stage_b_manager->addCamera(adas::Mount::SideCamL,
                                     g_ingest_manager->getCameraQueue(adas::Mount::SideCamL),
                                     g_det_side_l_queue);
    }
    if (mappings.find(adas::Mount::SideCamR) != mappings.end()) {
        g_stage_b_manager->addCamera(adas::Mount::SideCamR,
                                     g_ingest_manager->getCameraQueue(adas::Mount::SideCamR),
                                     g_det_side_r_queue);
    }
    */
    
    g_stage_b_manager->start();
    
    g_pipeline_running.store(true);
    g_pipeline_start_time = std::chrono::steady_clock::now();
    
    // Start status bar thread
    g_status_running.store(true);
    g_status_thread = std::thread(statusBarThread);
    
    // Start visualization thread
    g_visualizer_running.store(true);
    g_visualizer_thread = std::thread(visualizationThread);
    
    std::cout << "\n[Main] Pipeline started successfully!\n";
    std::cout << "[Main] Press '3' to view status, '2' to stop\n\n";
}

void stopPipeline() {
    if (!g_pipeline_running.load()) {
        std::cout << "[Main] Pipeline is not running\n";
        return;
    }
    
    std::cout << "\n[Main] Stopping pipeline...\n";
    
    // Stop status bar thread first
    g_status_running.store(false);
    if (g_status_thread.joinable()) {
        g_status_thread.join();
    }
    
    // Stop visualization thread
    g_visualizer_running.store(false);
    if (g_visualizer_thread.joinable()) {
        g_visualizer_thread.join();
    }
    
    // Stop in reverse order
    if (g_stage_b_manager) {
        g_stage_b_manager->stop();
        g_stage_b_manager.reset();
    }
    
    if (g_ingest_manager) {
        g_ingest_manager->stop();
        g_ingest_manager.reset();
    }
    
    g_pipeline_running.store(false);
    std::cout << "[Main] Pipeline stopped\n";
}

void showStatus() {
    if (!g_pipeline_running.load()) {
        std::cout << "\n[Main] Pipeline is not running\n";
        return;
    }
    
    if (g_ingest_manager) {
        g_ingest_manager->printStatus();
    }
    if (g_stage_b_manager) {
        g_stage_b_manager->printStatus();
    }
}

} // namespace

int main(int argc, char *argv[]) {
    printBanner();

    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Parse command line args
    std::string config_path = "config/componentConfig.yaml";
    std::string hw_map_path = "config/hardware_map.json";
    std::string calib_dir = "config/calibration";
    std::string model_path = "models/yolov5n.onnx";
    bool auto_start = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--hardware-map" && i + 1 < argc) {
            hw_map_path = argv[++i];
        } else if (arg == "--calib-dir" && i + 1 < argc) {
            calib_dir = argv[++i];
        } else if (arg == "--model" && i + 1 < argc) {
            model_path = argv[++i];
        } else if (arg == "--auto-start") {
            auto_start = true;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --config <path>        Path to componentConfig.yaml\n"
                      << "  --hardware-map <path>  Path to hardware_map.json\n"
                      << "  --calib-dir <path>     Path to calibration directory\n"
                      << "  --model <path>         Path to YOLOv8 ONNX model\n"
                      << "  --auto-start           Start pipeline automatically\n"
                      << "  --help                 Show this help\n";
            return 0;
        }
    }

    try {
        // Check if hardware map exists
        if (!adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
            std::cout << "[Main] Hardware map not found at " << hw_map_path << "\n";
            std::cout << "[Main] Please run Device Wizard first (option 4)\n\n";
        }

        // Load configuration
        std::cout << "[Main] Loading configuration from: " << config_path << "\n";
        adas::Config config = adas::ConfigLoader::loadConfig(config_path);

        // Load hardware mapping (may be empty if file doesn't exist)
        adas::HardwareMap hw_map;
        if (adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
            std::cout << "[Main] Loading hardware map from: " << hw_map_path << "\n";
            hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);
            
            std::cout << "[Main] Mapped devices:\n";
            for (const auto &[mount, path] : hw_map.mappings) {
                std::cout << "  " << adas::mountToString(mount) << " -> " << path << "\n";
            }
        }

        // Auto-start if requested
        if (auto_start && !hw_map.mappings.empty()) {
            startPipeline(config, hw_map, calib_dir, model_path);
        }

        // Interactive menu loop
        while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
            printMenu();
            
            int choice = -1;
            std::cin >> choice;
            
            if (std::cin.fail()) {
                std::cin.clear();
                std::cin.ignore(10000, '\n');
                continue;
            }
            
            switch (choice) {
                case 1:  // Start Pipeline
                    // Reload hw_map in case wizard was run
                    if (adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
                        hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);
                    }
                    if (hw_map.mappings.empty()) {
                        std::cout << "[Main] No devices mapped. Run Device Wizard first (option 4)\n";
                    } else {
                        startPipeline(config, hw_map, calib_dir, model_path);
                    }
                    break;
                    
                case 2:  // Stop Pipeline
                    stopPipeline();
                    break;
                    
                case 3:  // Show Status
                    showStatus();
                    break;
                    
                case 4:  // Device Wizard
                    if (g_pipeline_running.load()) {
                        std::cout << "[Main] Please stop the pipeline first\n";
                    } else {
                        adas::DeviceWizard::runRegistration(hw_map_path);
                    }
                    break;
                    
                case 5:  // Camera Calibration
                    if (g_pipeline_running.load()) {
                        std::cout << "[Main] Please stop the pipeline first\n";
                    } else {
                        // Reload hw_map and run calibration
                        if (adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
                            hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);
                        }
                        adas::DeviceWizard::runCalibration(hw_map, calib_dir);
                    }
                    break;
                    
                case 6:  // Register Pi4 Network Devices
                    if (g_pipeline_running.load()) {
                        std::cout << "[Main] Please stop the pipeline first\n";
                    } else {
                        // Hardcoded Pi IP for static ethernet-to-ethernet connection
                        adas::DeviceWizard::registerNetworkDevices(hw_map_path, "192.168.55.2");
                    }
                    break;
                    
                case 7:  // Test RTT to Pi4
                    {
                        std::cout << "  Enter Pi4 IP address: ";
                        std::string pi_ip;
                        std::cin >> pi_ip;
                        std::cin.ignore(10000, '\n');
                        double rtt = adas::DeviceWizard::measureRTT(pi_ip);
                        if (rtt < 0) {
                            std::cout << "[RTT] Failed to reach " << pi_ip << "\n";
                        } else {
                            std::cout << "[RTT] Round-trip time to " << pi_ip << ": " << rtt << " ms\n";
                            if (rtt < 10) {
                                std::cout << "[RTT] Status: EXCELLENT (< 10ms)\n";
                            } else if (rtt < 25) {
                                std::cout << "[RTT] Status: GOOD (< 25ms)\n";
                            } else if (rtt < 50) {
                                std::cout << "[RTT] Status: ACCEPTABLE (< 50ms)\n";
                            } else {
                                std::cout << "[RTT] Status: WARNING - High latency may affect sync\n";
                            }
                        }
                    }
                    break;
                    
                case 8:  // Toggle Verbose Mode
                    {
                        bool new_state = !adas::g_verbose_mode.load();
                        adas::g_verbose_mode.store(new_state);
                        std::cout << "[Main] Verbose mode " << (new_state ? "ENABLED" : "DISABLED") << "\n";
                    }
                    break;
                    
                case 0:  // Exit
                    stopPipeline();
                    std::cout << "\n[Main] Goodbye!\n";
                    return 0;
                    
                default:
                    std::cout << "[Main] Invalid choice\n";
                    break;
            }
        }

        // Graceful shutdown on signal
        stopPipeline();
        std::cout << "[Main] Shutdown complete.\n";
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "[Main] Fatal error: " << e.what() << "\n";
        return 1;
    }
}
