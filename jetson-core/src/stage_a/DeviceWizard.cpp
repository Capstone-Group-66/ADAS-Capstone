// File: src/stage_a/DeviceWizard.cpp
// Interactive device mapping implementation
#include "adas/stage_a/DeviceWizard.hpp"
#include "adas/stage_a/CameraCalibrator.hpp"

#ifdef HAS_ZMQ
#include "adas/stage_a/NetworkReceiver.hpp"
#endif

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>

namespace adas {

void DeviceWizard::runRegistration(const std::string &output_path,
                                   bool show_preview) {
  std::cout
      << "==============================================================\n";
  std::cout
      << "          ADAS Device Registration Wizard                     \n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "  This tool maps USB cameras to their mount positions.        \n";
  std::cout
      << "  Note: RearCam and Rear Radars come via Pi4 network.        \n";
  std::cout
      << "==============================================================\n\n";

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
    std::cout
        << "--------------------------------------------------------------\n";
    std::cout << "Device: " << device_path << "\n";

    if (device_path == "/dev/video0") {
      std::cout << "  [RESERVED] " << device_path << " -> FrontCam (DeepStream)\n";
      mappings[Mount::FrontCam] = device_path;
      continue;
    }

    // Test if device can be opened
    if (!testVideoDevice(device_path)) {
      std::cout << "  [SKIP] Cannot open device\n";
      continue;
    }

    // Open camera for continuous preview during selection
    cv::VideoCapture cap;
    std::string window_name;
    if (show_preview) {
#ifdef __linux__
      cap.open(device_path, cv::CAP_V4L2);
#else
      int device_num = std::stoi(device_path);
      cap.open(device_num);
#endif
      if (cap.isOpened()) {
        cap.set(cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        window_name =
            "Preview: " + device_path + " (make selection in terminal)";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

        // Show initial frame
        cv::Mat frame;
        if (cap.read(frame)) {
          cv::putText(frame, "Select mount in terminal...", cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
          cv::imshow(window_name, frame);
          cv::waitKey(1);
        }
      }
    }

    // Print options BEFORE blocking on input
    auto direct_mounts = getDirectCameraMounts();
    std::cout << "\n  Select mount for this device:\n";
    int option = 1;
    std::vector<Mount> available;
    for (Mount m : direct_mounts) {
      bool assigned =
          std::find(already_assigned.begin(), already_assigned.end(), m) !=
          already_assigned.end();
      if (!assigned) {
        available.push_back(m);
        std::cout << "    " << option << ") " << mountToString(m) << "\n";
        ++option;
      }
    }
    std::cout << "    0) Skip this device\n";
    std::cout << "  Enter choice: " << std::flush;

    // Keep updating preview while waiting for input (non-blocking check)
    int choice = -1;
    if (show_preview && cap.isOpened()) {
      // Use a simple polling approach - update preview every 100ms
      while (choice < 0) {
        cv::Mat frame;
        if (cap.read(frame)) {
          cv::putText(frame, "Select mount in terminal...", cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
          cv::imshow(window_name, frame);
        }
        cv::waitKey(100);

        // Check if input is available (non-blocking on Windows is tricky, use
        // blocking) For simplicity, just do blocking input after showing
        // preview
        if (std::cin.rdbuf()->in_avail() > 0 || true) {
          std::cin >> choice;
          break;
        }
      }
      cv::destroyWindow(window_name);
      cap.release();
    } else {
      std::cin >> choice;
    }

    // Process choice
    std::optional<Mount> mount = std::nullopt;
    if (choice > 0 && choice <= static_cast<int>(available.size())) {
      mount = available[choice - 1];
    }

    if (mount.has_value()) {
      mappings[mount.value()] = device_path;
      already_assigned.push_back(mount.value());
      std::cout << "  [ASSIGNED] " << device_path << " -> "
                << mountToString(mount.value()) << "\n";
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

void DeviceWizard::saveDirectMapping(
    const std::string &output_path,
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

void DeviceWizard::showPreview(const std::string &device_path,
                               int duration_sec) {
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
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >=
        duration_sec) {
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

std::optional<Mount> DeviceWizard::promptMountAssignment(
    const std::string &device_path,
    const std::vector<Mount> &already_assigned) {

  auto direct_mounts = getDirectCameraMounts();

  std::cout << "\n  Select mount for this device:\n";
  int option = 1;
  std::vector<Mount> available;

  for (Mount m : direct_mounts) {
    bool assigned = std::find(already_assigned.begin(), already_assigned.end(),
                              m) != already_assigned.end();
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
  // Only side cameras - RearCam comes via network, FrontCam used by DeepStream
  return {Mount::SideCamL, Mount::SideCamR};
}

void DeviceWizard::printSummary(const std::map<Mount, std::string> &mappings) {
  std::cout
      << "==============================================================\n";
  std::cout
      << "                    ASSIGNMENT SUMMARY                        \n";
  std::cout
      << "==============================================================\n";

  for (const auto &[mount, path] : mappings) {
    std::cout << "  " << std::left << std::setw(15) << mountToString(mount)
              << " -> " << std::setw(20) << path << "\n";
  }

  // Show unassigned
  auto direct_mounts = getDirectCameraMounts();
  for (Mount m : direct_mounts) {
    if (mappings.find(m) == mappings.end()) {
      std::cout << "  " << std::left << std::setw(15) << mountToString(m)
                << " -> " << std::setw(20) << "(not assigned)" << "\n";
    }
  }

  std::cout
      << "==============================================================\n";
  std::cout
      << "  Note: RearCam + RearRadars come via Pi4 (NetworkIngest)    \n";
  std::cout
      << "==============================================================\n";
}

std::string DeviceWizard::getCurrentTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now = *std::gmtime(&time_t_now);

  std::ostringstream oss;
  oss << std::put_time(&tm_now, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

void DeviceWizard::runCalibration(const HardwareMap &hw_map,
                                  const std::string &calib_dir,
                                  bool recalibrate) {
  std::cout << "\n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "                    CAMERA CALIBRATION                        \n";
  std::cout
      << "==============================================================\n";

  // Ensure calibration directory exists
  if (!std::filesystem::exists(calib_dir)) {
    std::filesystem::create_directories(calib_dir);
  }

  // Get camera mounts from hardware map
  std::vector<Mount> camera_mounts = {Mount::FrontCam, Mount::SideCamL,
                                      Mount::SideCamR, Mount::RearCam};

  int calibrated_count = 0;

  for (Mount mount : camera_mounts) {
    auto it = hw_map.mappings.find(mount);
    if (it == hw_map.mappings.end()) {
      continue; // Mount not mapped
    }

    std::string device_path = it->second;
    std::string calib_file =
        calib_dir + "/" + mountToString(mount) + "_calibration.yaml";

    // Check if calibration already exists
    if (!recalibrate && std::filesystem::exists(calib_file)) {
      std::cout << "[Calibration] " << mountToString(mount)
                << " already calibrated. Skipping.\n";
      continue;
    }

    std::cout << "\n";
    std::cout
        << "==============================================================\n";
    std::cout << "          Camera Calibration: " << mountToString(mount)
              << "\n";
    std::cout
        << "==============================================================\n";
    std::cout << "  Device: " << device_path << "\n";

    // Create calibrator and run
    CameraCalibrator calibrator;
    auto result = calibrator.runCalibration(device_path, mount, calib_dir);

    if (result.has_value()) {
      calibrated_count++;
    }
  }

  std::cout << "\n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "                 CALIBRATION COMPLETE                         \n";
  std::cout
      << "==============================================================\n";
  std::cout << "  Calibrated " << calibrated_count << " camera(s)\n";
  std::cout
      << "==============================================================\n";
}

void DeviceWizard::registerNetworkDevices(const std::string &hw_map_path,
                                          const std::string &pi_ip_arg) {
  std::cout << "\n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "          PI4 NETWORK DEVICE REGISTRATION                     \n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "  The Raspberry Pi 4 hosts the rear sector devices:           \n";
  std::cout
      << "    - RearCam (via ZMQ port 5555)                             \n";
  std::cout
      << "    - RearCornerRadarL (via ZMQ port 5556)                    \n";
  std::cout
      << "    - RearCornerRadarR (via ZMQ port 5557)                    \n";
  std::cout
      << "    - IMU (via ZMQ port 5558)                                 \n";
  std::cout
      << "==============================================================\n\n";

  // Get Pi IP
  std::string pi_ip = pi_ip_arg;
  if (pi_ip.empty()) {
    std::cout << "  Enter Pi4 IP address (e.g., 192.168.1.100): ";
    std::cin >> pi_ip;
    std::cin.ignore(10000, '\n');
  }

  if (pi_ip.empty()) {
    std::cerr << "\n[ERROR] No IP address provided.\n";
    return;
  }

  // Test connectivity
  std::cout << "\n[Network] Testing connectivity to " << pi_ip << "...\n";

#ifdef HAS_ZMQ
  // Use ZMQ-based RTT measurement (more accurate)
  double rtt = NetworkReceiver::measureRTT(pi_ip);
  if (rtt > 0) {
    std::cout << "[Network] ZMQ RTT to Pi: " << rtt << " ms\n";
  }
#else
  double rtt = measureRTT(pi_ip);
#endif

  if (rtt < 0) {
    std::cerr << "\n[WARNING] Could not reach Pi at " << pi_ip << "\n";
    std::cout << "  Continue anyway? (y/N): ";
    std::string response;
    std::getline(std::cin, response);
    if (response != "y" && response != "Y") {
      return;
    }
  } else {
    if (rtt < 10) {
      std::cout << "[Network] Status: EXCELLENT (RTT < 10ms)\n";
    } else if (rtt < 25) {
      std::cout << "[Network] Status: GOOD (RTT < 25ms)\n";
    } else if (rtt < 50) {
      std::cout << "[Network] Status: ACCEPTABLE (RTT < 50ms)\n";
    } else {
      std::cout << "[Network] Status: WARNING - High latency\n";
    }
  }

#ifdef HAS_ZMQ
  // Try ZMQ device discovery
  std::cout << "\n[Network] Attempting device discovery via ZMQ...\n";
  auto devices = NetworkReceiver::discoverDevices(pi_ip, 3000);

  if (!devices.empty()) {
    std::cout << "[Network] Discovered " << devices.size() << " devices:\n";
    for (const auto &dev : devices) {
      std::string type_str = dev.type == protocol::DeviceType::CAMERA ? "Camera"
                             : dev.type == protocol::DeviceType::RADAR ? "Radar"
                                                                       : "IMU";
      std::string status_str =
          dev.status == protocol::DeviceStatus::OK ? "OK" : "ERROR";
      std::cout << "  - " << type_str << " (" << dev.serial << ") - "
                << status_str << "\n";
    }
  } else {
    std::cout << "[Network] Discovery failed or Pi not responding.\n";
    std::cout << "[Network] Registering default Pi devices anyway...\n";
  }
#else
  std::cout << "[Network] ZMQ not available, registering default devices.\n";
#endif

  // Load existing hardware map or create new one
  HardwareMap hw_map;
  if (std::filesystem::exists(hw_map_path)) {
    try {
      hw_map = ConfigLoader::loadHardwareMap(hw_map_path);
      std::cout << "\n[Network] Loaded existing hardware map\n";
    } catch (...) {
      std::cout << "\n[Network] Creating new hardware map\n";
    }
  }

  hw_map.schema_version = "1.0";
  hw_map.generated_at = getCurrentTimestamp();

  // Build network device addresses
  // Format: "zmq://IP:PORT" for network devices
  std::string rear_cam_addr = "zmq://" + pi_ip + ":5555";
  std::string radar_l_addr = "zmq://" + pi_ip + ":5556";
  std::string radar_r_addr = "zmq://" + pi_ip + ":5557";
  std::string imu_addr = "zmq://" + pi_ip + ":5558";

  // Add network devices
  hw_map.mappings[Mount::RearCam] = rear_cam_addr;
  hw_map.mappings[Mount::RearCornerRadarL] = radar_l_addr;
  hw_map.mappings[Mount::RearCornerRadarR] = radar_r_addr;
  hw_map.mappings[Mount::IMU] = imu_addr;

  // Save updated hardware map
  ConfigLoader::saveHardwareMap(hw_map_path, hw_map);

  std::cout << "\n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "             NETWORK DEVICES REGISTERED                       \n";
  std::cout
      << "==============================================================\n";
  std::cout << "  RearCam          -> " << rear_cam_addr << "\n";
  std::cout << "  RearCornerRadarL -> " << radar_l_addr << "\n";
  std::cout << "  RearCornerRadarR -> " << radar_r_addr << "\n";
  std::cout << "  IMU              -> " << imu_addr << "\n";
  std::cout
      << "==============================================================\n";
  std::cout << "\nSaved to: " << hw_map_path << "\n";
}

double DeviceWizard::measureRTT(const std::string &pi_ip) {
#ifdef HAS_ZMQ
  // Use ZMQ-based RTT measurement
  return NetworkReceiver::measureRTT(pi_ip);
#else
  // Fallback to ping-based RTT measurement
#ifdef _WIN32
  std::string cmd = "ping -n 1 -w 1000 " + pi_ip + " > nul 2>&1";
#else
  std::string cmd = "ping -c 1 -W 1 " + pi_ip + " > /dev/null 2>&1";
#endif

  auto start = std::chrono::high_resolution_clock::now();
  int result = std::system(cmd.c_str());
  auto end = std::chrono::high_resolution_clock::now();

  if (result != 0) {
    return -1.0; // Ping failed
  }

  double rtt_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  return rtt_ms;
#endif
}

} // namespace adas
