// File: src/device_wizard_main.cpp
// Standalone DeviceWizard executable with calibration support
#include "adas/common/Config.hpp"
#include "adas/stage_a/CameraCalibrator.hpp"
#include "adas/stage_a/DeviceWizard.hpp"

#include <iostream>

void printUsage(const char *program_name) {
    std::cout << "Usage: " << program_name << " [options]\n"
              << "\nDevice Registration:\n"
              << "  --output <path>         Output path for hardware_map.json (default: "
                 "config/hardware_map.json)\n"
              << "  --no-preview            Skip camera preview windows during registration\n"
              << "\nCamera Calibration:\n"
              << "  --calibrate             Run camera calibration after registration\n"
              << "  --recalibrate           Force recalibration even if calibration files exist\n"
              << "  --calibration-dir <dir> Directory for calibration files (default: "
                 "config/calibration/)\n"
              << "  --pattern-size <WxH>    Chessboard inner corners (default: 9x6)\n"
              << "  --square-size <meters>  Square size in meters (default: 0.025)\n"
              << "\nOther:\n"
              << "  --help                  Show this help\n";
}

int main(int argc, char *argv[]) {
    std::string output_path = "config/hardware_map.json";
    std::string calibration_dir = "config/calibration";
    bool show_preview = true;
    bool run_calibration = false;
    bool force_recalibrate = false;
    int pattern_w = adas::CameraCalibrator::DEFAULT_PATTERN_WIDTH;
    int pattern_h = adas::CameraCalibrator::DEFAULT_PATTERN_HEIGHT;
    double square_size = adas::CameraCalibrator::DEFAULT_SQUARE_SIZE_M;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--no-preview") {
            show_preview = false;
        } else if (arg == "--calibrate") {
            run_calibration = true;
        } else if (arg == "--recalibrate") {
            run_calibration = true;
            force_recalibrate = true;
        } else if (arg == "--calibration-dir" && i + 1 < argc) {
            calibration_dir = argv[++i];
        } else if (arg == "--pattern-size" && i + 1 < argc) {
            std::string size_str = argv[++i];
            size_t x_pos = size_str.find('x');
            if (x_pos != std::string::npos) {
                pattern_w = std::stoi(size_str.substr(0, x_pos));
                pattern_h = std::stoi(size_str.substr(x_pos + 1));
            }
        } else if (arg == "--square-size" && i + 1 < argc) {
            square_size = std::stod(argv[++i]);
        } else if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    // Step 1: Run device registration
    adas::DeviceWizard::runRegistration(output_path, show_preview);

    // Step 2: Run calibration if requested
    if (run_calibration) {
        std::cout << "\n";
        std::cout << "==============================================================\n";
        std::cout << "                    CAMERA CALIBRATION                        \n";
        std::cout << "==============================================================\n";

        // Load hardware map to get registered cameras
        auto hw_map = adas::ConfigLoader::loadHardwareMap(output_path);
        if (!hw_map.has_value()) {
            std::cerr << "ERROR: Could not load hardware map from " << output_path << "\n";
            std::cerr << "Run device registration first.\n";
            return 1;
        }

        adas::CameraCalibrator calibrator(cv::Size(pattern_w, pattern_h), square_size);

        // Calibrate each registered camera
        for (const auto &[mount, device_path] : hw_map->mappings) {
            // Check if calibration already exists
            bool exists = adas::CameraCalibrator::calibrationExists(mount, calibration_dir);

            if (exists && !force_recalibrate) {
                std::cout << "\n[" << adas::mountToString(mount)
                          << "] Calibration exists, skipping.\n";
                std::cout << "  (Use --recalibrate to force recalibration)\n";
                continue;
            }

            // Run calibration
            auto result = calibrator.runCalibration(device_path, mount, calibration_dir);

            if (!result.has_value()) {
                std::cerr << "[" << adas::mountToString(mount)
                          << "] Calibration failed or cancelled.\n";
            }
        }

        std::cout << "\n";
        std::cout << "==============================================================\n";
        std::cout << "                 CALIBRATION COMPLETE                         \n";
        std::cout << "==============================================================\n";
    }

    return 0;
}
