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

namespace {

std::unique_ptr<adas::IngestManager> g_manager;
std::atomic<bool> g_shutdown_requested{false};

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
                                                                       
                Stage A: Ingest & Timestamp Pipeline                   
                                                                       
    ===========================================================================
    )" << std::endl;
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
    bool run_wizard = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--hardware-map" && i + 1 < argc) {
            hw_map_path = argv[++i];
        } else if (arg == "--wizard") {
            run_wizard = true;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --config <path>        Path to componentConfig.yaml\n"
                      << "  --hardware-map <path>  Path to hardware_map.json\n"
                      << "  --wizard               Run device registration wizard\n"
                      << "  --help                 Show this help\n";
            return 0;
        }
    }

    try {
        // Check if hardware map exists, run wizard if not
        if (!adas::ConfigLoader::hardwareMapExists(hw_map_path) || run_wizard) {
            std::cout << "[Main] Hardware map not found or wizard requested\n";
            std::cout << "[Main] Running Device Registration Wizard...\n\n";
            adas::DeviceWizard::runRegistration(hw_map_path);
        }

        // Load configuration
        std::cout << "[Main] Loading configuration from: " << config_path << "\n";
        adas::Config config = adas::ConfigLoader::loadConfig(config_path);

        // Load hardware mapping
        std::cout << "[Main] Loading hardware map from: " << hw_map_path << "\n";
        adas::HardwareMap hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);

        std::cout << "[Main] Mapped devices:\n";
        for (const auto &[mount, path] : hw_map.mappings) {
            std::cout << "  " << adas::mountToString(mount) << " -> " << path << "\n";
        }

        // Create and start IngestManager
        g_manager = std::make_unique<adas::IngestManager>(config, hw_map);
        g_manager->start();

        adas::SensorFusion sensorFusion(g_manager->getRadarQueue(adas::Mount::FrontRadar), g_manager->getIMUQueue());
        sensorFusion.start();

        // Main loop - print status periodically
        uint64_t last_status = adas::Clock::now_ns();
        while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Print status every 10 seconds
            if (adas::Clock::elapsed_ms(last_status) >= 10000) {
                g_manager->printStatus();
                last_status = adas::Clock::now_ns();
            }
        }

        // Graceful shutdown (FR93)
        std::cout << "\n[Main] Shutting down Stage A pipeline...\n";
        g_manager->stop();
        g_manager.reset();
        sensorFusion.stop();

        std::cout << "[Main] Shutdown complete.\n";
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "[Main] Fatal error: " << e.what() << "\n";
        return 1;
    }
}
