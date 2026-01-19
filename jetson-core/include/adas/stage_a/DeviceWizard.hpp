// File: include/adas/stage_a/DeviceWizard.hpp
// Interactive device mapping CLI for non-deterministic USB enumeration
#pragma once

#include "adas/common/Config.hpp"
#include "adas/common/Types.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace adas {

/// DeviceWizard: Interactive CLI for mapping /dev/videoX devices to Mounts
/// Resolves the issue of non-deterministic USB device enumeration on Linux
/// Run this tool before starting the pipeline to generate hardware_map.json
class DeviceWizard {
  public:
    /// Run full interactive registration process
    /// @param output_path Path to save hardware_map.json
    /// @param show_preview If true, show camera preview window (requires display)
    static void runRegistration(const std::string &output_path, bool show_preview = true);

    /// Run non-interactive registration with provided mappings
    /// Used for automated testing or scripted setups
    /// @param output_path Path to save hardware_map.json
    /// @param mappings Pre-defined mount to device path mappings
    static void saveDirectMapping(const std::string &output_path,
                                  const std::map<Mount, std::string> &mappings);

    /// Run camera calibration for all mapped cameras
    /// @param hw_map Hardware mapping from previous registration
    /// @param calib_dir Directory to save calibration files
    /// @param recalibrate If true, recalibrate even if calibration exists
    static void runCalibration(const HardwareMap& hw_map, 
                               const std::string& calib_dir = "config/calibration",
                               bool recalibrate = false);

    /// Enumerate all /dev/video* devices
    /// @return List of available video device paths
    static std::vector<std::string> enumerateVideoDevices();

    /// Enumerate serial devices (for radar)
    /// @return List of available serial device paths (/dev/ttyACM*, /dev/ttyUSB*)
    static std::vector<std::string> enumerateSerialDevices();

    /// Test if a video device can be opened
    /// @param device_path Path to video device
    /// @return true if device opens successfully
    static bool testVideoDevice(const std::string &device_path);

    /// Show preview window for a video device
    /// @param device_path Path to video device
    /// @param duration_sec How long to show preview
    static void showPreview(const std::string &device_path, int duration_sec = 3);

    /// Get current timestamp in ISO 8601 format
    static std::string getCurrentTimestamp();

    /// Register Pi4 network devices (RearCam, RearRadarL, RearRadarR)
    /// @param hw_map_path Path to hardware_map.json (will merge with existing)
    /// @param pi_ip IP address of Pi4 (e.g., "192.168.1.100")
    static void registerNetworkDevices(const std::string& hw_map_path, 
                                        const std::string& pi_ip = "");

    /// Test RTT (round-trip time) to Pi4
    /// @param pi_ip IP address of Pi4
    /// @return RTT in milliseconds, or -1 if failed
    static double measureRTT(const std::string& pi_ip);

  private:
    /// Prompt user to assign a mount to a device
    /// @param device_path Device being assigned
    /// @param already_assigned Mounts that have already been assigned
    /// @return Selected mount, or nullopt if skipped
    static std::optional<Mount> promptMountAssignment(const std::string &device_path,
                                                      const std::vector<Mount> &already_assigned);

    /// Get list of camera mounts that need to be assigned
    static std::vector<Mount> getDirectCameraMounts();

    /// Print assignment summary
    static void printSummary(const std::map<Mount, std::string> &mappings);
};

} // namespace adas
