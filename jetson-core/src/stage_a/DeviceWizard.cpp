// File: src/stage_a/DeviceWizard.cpp
// Interactive device mapping implementation
#include "adas/stage_a/DeviceWizard.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace adas {

void DeviceWizard::runRegistration(const std::string &output_path, bool show_preview) {
    std::cout << "==============================================================\n";
    std::cout << "          ADAS Device Registration Wizard                     \n";
    std::cout << "==============================================================\n";
    std::cout << "  This tool maps USB cameras to their mount positions.        \n";
    std::cout << "  Note: RearCam and Rear Radars come via Pi4 network.        \n";
    std::cout << "==============================================================\n\n";

    // Enumerate devices
    auto video_devices = enumerateVideoDevices();

    if (video_devices.empty()) {
        std::cerr << "ERROR: No video devices found!\n";
        std::cerr << "Make sure cameras are connected and accessible.\n";
        return;
    }

    std::cout << "Found " << video_devices.size() << " video device(s):\n";
    for (const auto &dev : video_devices) {
        std::cout << "  - " << dev << "\n";
    }
    std::cout << "\n";

    // Mounts that need assignment (direct USB cameras only)
    std::vector<Mount> direct_mounts = getDirectCameraMounts();
    std::map<Mount, std::string> mappings;
    std::vector<Mount> already_assigned;

    // Process each device
    for (const auto &device_path : video_devices) {
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "Device: " << device_path << "\n";

        // Test if device can be opened
        if (!testVideoDevice(device_path)) {
            std::cout << "  [SKIP] Cannot open device\n";
            continue;
        }

        // Show preview if requested
        if (show_preview) {
            std::cout << "  Showing preview (3 seconds)...\n";
            showPreview(device_path, 3);
        }

        // Prompt for assignment
        auto mount = promptMountAssignment(device_path, already_assigned);
        if (mount.has_value()) {
            mappings[mount.value()] = device_path;
            already_assigned.push_back(mount.value());
            std::cout << "  [ASSIGNED] " << device_path << " -> " << mountToString(mount.value())
                      << "\n";
        } else {
            std::cout << "  [SKIPPED]\n";
        }

        // Check if all mounts assigned
        if (already_assigned.size() == direct_mounts.size()) {
            std::cout << "\nAll direct camera mounts have been assigned.\n";
            break;
        }
    }

    // Summary
    std::cout << "\n";
    printSummary(mappings);

    // Save mapping
    if (!mappings.empty()) {
        HardwareMap hw_map;
        hw_map.schema_version = "1.0";
        hw_map.generated_at = getCurrentTimestamp();
        hw_map.mappings = mappings;

        ConfigLoader::saveHardwareMap(output_path, hw_map);
        std::cout << "\nSaved mapping to: " << output_path << "\n";
    } else {
        std::cerr << "\nWARNING: No devices were assigned!\n";
    }
}

void DeviceWizard::saveDirectMapping(const std::string &output_path,
                                     const std::map<Mount, std::string> &mappings) {
    HardwareMap hw_map;
    hw_map.schema_version = "1.0";
    hw_map.generated_at = getCurrentTimestamp();
    hw_map.mappings = mappings;

    ConfigLoader::saveHardwareMap(output_path, hw_map);
}

std::vector<std::string> DeviceWizard::enumerateVideoDevices() {
    std::vector<std::string> devices;

#ifdef __linux__
    // Linux: scan /dev/video*
    for (int i = 0; i < 20; ++i) {
        std::string path = "/dev/video" + std::to_string(i);
        if (std::filesystem::exists(path)) {
            devices.push_back(path);
        }
    }
#else
    // Windows/other: just try indices 0-10
    for (int i = 0; i < 10; ++i) {
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            devices.push_back(std::to_string(i));
            cap.release();
        }
    }
#endif

    return devices;
}

std::vector<std::string> DeviceWizard::enumerateSerialDevices() {
    std::vector<std::string> devices;

#ifdef __linux__
    // Linux: scan /dev/ttyACM* and /dev/ttyUSB*
    for (int i = 0; i < 10; ++i) {
        std::string path = "/dev/ttyACM" + std::to_string(i);
        if (std::filesystem::exists(path)) {
            devices.push_back(path);
        }
    }
    for (int i = 0; i < 10; ++i) {
        std::string path = "/dev/ttyUSB" + std::to_string(i);
        if (std::filesystem::exists(path)) {
            devices.push_back(path);
        }
    }
#endif

    return devices;
}

bool DeviceWizard::testVideoDevice(const std::string &device_path) {
#ifdef __linux__
    cv::VideoCapture cap(device_path, cv::CAP_V4L2);
#else
    int device_num = std::stoi(device_path);
    cv::VideoCapture cap(device_num);
#endif

    if (!cap.isOpened()) {
        return false;
    }

    // Try to read a frame
    cv::Mat frame;
    bool success = cap.read(frame);
    cap.release();

    return success && !frame.empty();
}

void DeviceWizard::showPreview(const std::string &device_path, int duration_sec) {
#ifdef __linux__
    cv::VideoCapture cap(device_path, cv::CAP_V4L2);
#else
    int device_num = std::stoi(device_path);
    cv::VideoCapture cap(device_num);
#endif

    if (!cap.isOpened()) {
        std::cerr << "  Cannot open device for preview\n";
        return;
    }

    // Force MJPEG
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    std::string window_name = "Preview: " + device_path;
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    auto start = std::chrono::steady_clock::now();
    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            break;
        }

        cv::imshow(window_name, frame);

        // Check duration
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= duration_sec) {
            break;
        }

        // Allow early exit with key press
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }

    cv::destroyWindow(window_name);
    cap.release();
}

std::optional<Mount>
DeviceWizard::promptMountAssignment(const std::string &device_path,
                                    const std::vector<Mount> &already_assigned) {

    auto direct_mounts = getDirectCameraMounts();

    std::cout << "\n  Select mount for this device:\n";
    int option = 1;
    std::vector<Mount> available;

    for (Mount m : direct_mounts) {
        bool assigned = std::find(already_assigned.begin(), already_assigned.end(), m) !=
                        already_assigned.end();
        if (!assigned) {
            available.push_back(m);
            std::cout << "    " << option << ") " << mountToString(m) << "\n";
            ++option;
        }
    }
    std::cout << "    0) Skip this device\n";
    std::cout << "  Enter choice: ";

    int choice;
    std::cin >> choice;

    if (choice <= 0 || choice > static_cast<int>(available.size())) {
        return std::nullopt;
    }

    return available[choice - 1];
}

std::vector<Mount> DeviceWizard::getDirectCameraMounts() {
    // Only direct USB cameras - RearCam comes via network
    return {Mount::FrontCam, Mount::SideCamL, Mount::SideCamR};
}

void DeviceWizard::printSummary(const std::map<Mount, std::string> &mappings) {
    std::cout << "==============================================================\n";
    std::cout << "                    ASSIGNMENT SUMMARY                        \n";
    std::cout << "==============================================================\n";

    for (const auto &[mount, path] : mappings) {
        std::cout << "  " << std::left << std::setw(15) << mountToString(mount) << " -> "
                  << std::setw(20) << path << "\n";
    }

    // Show unassigned
    auto direct_mounts = getDirectCameraMounts();
    for (Mount m : direct_mounts) {
        if (mappings.find(m) == mappings.end()) {
            std::cout << "  " << std::left << std::setw(15) << mountToString(m) << " -> "
                      << std::setw(20) << "(not assigned)" << "\n";
        }
    }

    std::cout << "==============================================================\n";
    std::cout << "  Note: RearCam + RearRadars come via Pi4 (NetworkIngest)    \n";
    std::cout << "==============================================================\n";
}

std::string DeviceWizard::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::gmtime(&time_t_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace adas
