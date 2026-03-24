// File: src/common/Config.cpp
// Configuration loader implementation
#include "adas/common/Config.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

// Simple JSON parsing (minimal implementation without external dependency)
// For production, consider using nlohmann/json
namespace {

std::string trim(const std::string &s) {
  size_t start = s.find_first_not_of(" \t\n\r");
  if (start == std::string::npos) {
    return "";
  }
  size_t end = s.find_last_not_of(" \t\n\r");
  return s.substr(start, end - start + 1);
}

std::string readFile(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file: " + path);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::string normalizeRadarOutputMode(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "combined_native") {
    return "combined_native";
  }
  return "split_range";
}

} // namespace

namespace adas {

Config ConfigLoader::loadConfig(const std::string &path) {
  Config config;

  std::string content = readFile(path);

  // Simple YAML parsing for our known structure
  // In production, use yaml-cpp library
  std::istringstream stream(content);
  std::string line;
  std::string current_section; // Track which section we're in
  std::string current_mount;   // Track which mount subsection we're in

  while (std::getline(stream, line)) {
    // Check indentation to determine if this is a section header
    size_t indent = line.find_first_not_of(" \t");
    std::string trimmed = trim(line);

    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    // Parse key-value pairs
    size_t colonPos = trimmed.find(':');
    if (colonPos == std::string::npos) {
      continue;
    }

    std::string key = trim(trimmed.substr(0, colonPos));
    std::string value = trim(trimmed.substr(colonPos + 1));

    // If value is empty, this is a section header
    if (value.empty() || value[0] == '#') {
      if (indent == 0) {
        current_section = key;
        current_mount = "";
      } else if (current_section == "mounts" && indent > 0) {
        current_mount = key;
      }
      continue;
    }

    // Remove quotes from string values
    if (!value.empty() && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }

    // Time config
    if (current_section == "time") {
      if (key == "fusion_hz") {
        config.time.fusion_hz = std::stoi(value);
      } else if (key == "max_skew_ms") {
        config.time.max_skew_ms = std::stoi(value);
      } else if (key == "buffer_ms") {
        config.time.buffer_ms = std::stoi(value);
      } else if (key == "late_drop_ms") {
        config.time.late_drop_ms = std::stoi(value);
      } else if (key == "warmup_ticks") {
        config.time.warmup_ticks = std::stoi(value);
      }
    }
    // Camera config
    else if (current_section == "cameras") {
      if (key == "width") {
        config.cameras.width = std::stoi(value);
      } else if (key == "height") {
        config.cameras.height = std::stoi(value);
      } else if (key == "target_fps") {
        config.cameras.target_fps = std::stoi(value);
      } else if (key == "use_mjpeg") {
        config.cameras.use_mjpeg = (value == "true");
      }
    }
    // Network config
    else if (current_section == "network") {
      if (key == "port") {
        config.network.port = std::stoi(value);
      } else if (key == "latency_correction_ms") {
        config.network.latency_correction_ms = std::stoi(value);
      } else if (key == "reconnect_timeout_ms") {
        config.network.reconnect_timeout_ms = std::stoi(value);
      }
    }
    // Front radar config
    else if (current_section == "front_radar") {
      if (key == "port") {
        config.front_radar.port = value; // String, not int!
      } else if (key == "baud_rate") {
        config.front_radar.baud_rate = std::stoi(value);
      } else if (key == "poll_timeout_ms") {
        config.front_radar.poll_timeout_ms = std::stoi(value);
      } else if (key == "output_mode") {
        config.front_radar.output_mode = normalizeRadarOutputMode(value);
      } else if (key == "speed_ttl_ms") {
        config.front_radar.speed_ttl_ms = std::stoi(value);
      } else if (key == "speed_mag_threshold") {
        config.front_radar.speed_mag_threshold = std::stoi(value);
      } else if (key == "range_mag_threshold") {
        config.front_radar.range_mag_threshold = std::stoi(value);
      } else if (key == "profile") {
        config.front_radar.profile = value;
      }
    }
    // Stage E fusion config
    else if (current_section == "stage_e_fusion") {
      if (key == "ttc_aggressive_s") {
        config.stage_e_fusion.ttc_aggressive_s = std::stof(value);
      } else if (key == "camera_hold_ms") {
        config.stage_e_fusion.camera_hold_ms = std::stoi(value);
      } else if (key == "radar_hold_ms") {
        config.stage_e_fusion.radar_hold_ms = std::stoi(value);
      } else if (key == "track_cleanup_ms") {
        config.stage_e_fusion.track_cleanup_ms = std::stoi(value);
      } else if (key == "predicted_camera_threshold_ms") {
        config.stage_e_fusion.predicted_camera_threshold_ms = std::stoi(value);
      } else if (key == "gps_correction_gain") {
        config.stage_e_fusion.gps_correction_gain = std::stof(value);
      } else if (key == "normal_angle_gate_deg") {
        config.stage_e_fusion.normal_angle_gate_deg = std::stof(value);
      } else if (key == "aggressive_angle_gate_deg") {
        config.stage_e_fusion.aggressive_angle_gate_deg = std::stof(value);
      } else if (key == "aggressive_range_scale") {
        config.stage_e_fusion.aggressive_range_scale = std::stof(value);
      } else if (key == "ekf_q_z") {
        config.stage_e_fusion.ekf_q_z = std::stof(value);
      } else if (key == "ekf_q_vz") {
        config.stage_e_fusion.ekf_q_vz = std::stof(value);
      } else if (key == "ekf_q_theta") {
        config.stage_e_fusion.ekf_q_theta = std::stof(value);
      } else if (key == "ekf_q_theta_dot") {
        config.stage_e_fusion.ekf_q_theta_dot = std::stof(value);
      } else if (key == "ekf_r_radar_z") {
        config.stage_e_fusion.ekf_r_radar_z = std::stof(value);
      } else if (key == "ekf_r_radar_vz") {
        config.stage_e_fusion.ekf_r_radar_vz = std::stof(value);
      } else if (key == "ekf_r_cam_theta") {
        config.stage_e_fusion.ekf_r_cam_theta = std::stof(value);
      } else if (key == "ekf_r_cam_z_weak") {
        config.stage_e_fusion.ekf_r_cam_z_weak = std::stof(value);
      } else if (key == "derived_speed_min_hits") {
        config.stage_e_fusion.derived_speed_min_hits = std::stoi(value);
      } else if (key == "derived_speed_min_dt_ms") {
        config.stage_e_fusion.derived_speed_min_dt_ms = std::stoi(value);
      } else if (key == "derived_speed_max_dt_ms") {
        config.stage_e_fusion.derived_speed_max_dt_ms = std::stoi(value);
      } else if (key == "derived_speed_hold_ms") {
        config.stage_e_fusion.derived_speed_hold_ms = std::stoi(value);
      } else if (key == "derived_speed_max_plausible_mps") {
        config.stage_e_fusion.derived_speed_max_plausible_mps =
            std::stof(value);
      } else if (key == "derived_speed_jump_base_m") {
        config.stage_e_fusion.derived_speed_jump_base_m = std::stof(value);
      } else if (key == "derived_speed_jump_slope_mps") {
        config.stage_e_fusion.derived_speed_jump_slope_mps =
            std::stof(value);
      } else if (key == "radar_speed_disagreement_gate_mps") {
        config.stage_e_fusion.radar_speed_disagreement_gate_mps =
            std::stof(value);
      }
    }
    // IMU config
    else if (current_section == "imu") {
      if (key == "rate_hz") {
        config.imu.rate_hz = std::stoi(value);
      } else if (key == "use_uart") {
        config.imu.use_uart = (value == "true");
      } else if (key == "bus") {
        config.imu.bus = value;
      }
    }
    // Mounts config
    else if (current_section == "mounts" && !current_mount.empty()) {
      if (key == "xyz_m" && current_mount == "FrontCam") {
        if (value.size() >= 2 && value.front() == '[' && value.back() == ']') {
          std::string inner = value.substr(1, value.size() - 2);
          std::stringstream ss(inner);
          std::string item;
          int i = 0;
          while (std::getline(ss, item, ',') && i < 3) {
            config.mounts[Mount::FrontCam].xyz_m[i] = std::stof(trim(item));
            i++;
          }
        }
      }
    }
  }

  config.front_radar.output_mode =
      normalizeRadarOutputMode(config.front_radar.output_mode);
  return config;
}

void ConfigLoader::saveConfig(const std::string &path, const Config &config) {
  std::string content = readFile(path);
  std::istringstream stream(content);
  std::string line;
  std::string current_section;
  std::string current_mount;
  std::string new_content;

  while (std::getline(stream, line)) {
    size_t indent = line.find_first_not_of(" \t");
    std::string trimmed = trim(line);

    if (trimmed.empty() || trimmed[0] == '#') {
      new_content += line + "\n";
      continue;
    }

    size_t colonPos = trimmed.find(':');
    if (colonPos == std::string::npos) {
      new_content += line + "\n";
      continue;
    }

    std::string key = trim(trimmed.substr(0, colonPos));
    std::string value = trim(trimmed.substr(colonPos + 1));

    if (value.empty() || value[0] == '#') {
      if (indent == 0) {
        current_section = key;
        current_mount = "";
      } else if (current_section == "mounts" && indent > 0) {
        current_mount = key;
      }
      new_content += line + "\n";
      continue;
    }

    if (current_section == "stage_e_fusion") {
      auto emitFloat = [&](const std::string &name, float v) {
        std::stringstream ss;
        ss << std::string(indent, ' ') << name << ": " << v << "\n";
        new_content += ss.str();
      };

      if (key == "ttc_aggressive_s") {
        emitFloat("ttc_aggressive_s", config.stage_e_fusion.ttc_aggressive_s);
        continue;
      }
      if (key == "camera_hold_ms") {
        std::stringstream ss;
        ss << std::string(indent, ' ')
           << "camera_hold_ms: " << config.stage_e_fusion.camera_hold_ms
           << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "radar_hold_ms") {
        std::stringstream ss;
        ss << std::string(indent, ' ')
           << "radar_hold_ms: " << config.stage_e_fusion.radar_hold_ms << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "track_cleanup_ms") {
        std::stringstream ss;
        ss << std::string(indent, ' ')
           << "track_cleanup_ms: " << config.stage_e_fusion.track_cleanup_ms
           << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "predicted_camera_threshold_ms") {
        std::stringstream ss;
        ss << std::string(indent, ' ') << "predicted_camera_threshold_ms: "
           << config.stage_e_fusion.predicted_camera_threshold_ms << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "gps_correction_gain") {
        emitFloat("gps_correction_gain",
                  config.stage_e_fusion.gps_correction_gain);
        continue;
      }
      if (key == "normal_angle_gate_deg") {
        emitFloat("normal_angle_gate_deg",
                  config.stage_e_fusion.normal_angle_gate_deg);
        continue;
      }
      if (key == "aggressive_angle_gate_deg") {
        emitFloat("aggressive_angle_gate_deg",
                  config.stage_e_fusion.aggressive_angle_gate_deg);
        continue;
      }
      if (key == "aggressive_range_scale") {
        emitFloat("aggressive_range_scale",
                  config.stage_e_fusion.aggressive_range_scale);
        continue;
      }
      if (key == "ekf_q_z") {
        emitFloat("ekf_q_z", config.stage_e_fusion.ekf_q_z);
        continue;
      }
      if (key == "ekf_q_vz") {
        emitFloat("ekf_q_vz", config.stage_e_fusion.ekf_q_vz);
        continue;
      }
      if (key == "ekf_q_theta") {
        emitFloat("ekf_q_theta", config.stage_e_fusion.ekf_q_theta);
        continue;
      }
      if (key == "ekf_q_theta_dot") {
        emitFloat("ekf_q_theta_dot", config.stage_e_fusion.ekf_q_theta_dot);
        continue;
      }
      if (key == "ekf_r_radar_z") {
        emitFloat("ekf_r_radar_z", config.stage_e_fusion.ekf_r_radar_z);
        continue;
      }
      if (key == "ekf_r_radar_vz") {
        emitFloat("ekf_r_radar_vz", config.stage_e_fusion.ekf_r_radar_vz);
        continue;
      }
      if (key == "ekf_r_cam_theta") {
        emitFloat("ekf_r_cam_theta", config.stage_e_fusion.ekf_r_cam_theta);
        continue;
      }
      if (key == "ekf_r_cam_z_weak") {
        emitFloat("ekf_r_cam_z_weak", config.stage_e_fusion.ekf_r_cam_z_weak);
        continue;
      }
      if (key == "derived_speed_min_hits") {
        std::stringstream ss;
        ss << std::string(indent, ' ') << "derived_speed_min_hits: "
           << config.stage_e_fusion.derived_speed_min_hits << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "derived_speed_min_dt_ms") {
        std::stringstream ss;
        ss << std::string(indent, ' ') << "derived_speed_min_dt_ms: "
           << config.stage_e_fusion.derived_speed_min_dt_ms << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "derived_speed_max_dt_ms") {
        std::stringstream ss;
        ss << std::string(indent, ' ') << "derived_speed_max_dt_ms: "
           << config.stage_e_fusion.derived_speed_max_dt_ms << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "derived_speed_hold_ms") {
        std::stringstream ss;
        ss << std::string(indent, ' ') << "derived_speed_hold_ms: "
           << config.stage_e_fusion.derived_speed_hold_ms << "\n";
        new_content += ss.str();
        continue;
      }
      if (key == "derived_speed_max_plausible_mps") {
        emitFloat("derived_speed_max_plausible_mps",
                  config.stage_e_fusion.derived_speed_max_plausible_mps);
        continue;
      }
      if (key == "derived_speed_jump_base_m") {
        emitFloat("derived_speed_jump_base_m",
                  config.stage_e_fusion.derived_speed_jump_base_m);
        continue;
      }
      if (key == "derived_speed_jump_slope_mps") {
        emitFloat("derived_speed_jump_slope_mps",
                  config.stage_e_fusion.derived_speed_jump_slope_mps);
        continue;
      }
      if (key == "radar_speed_disagreement_gate_mps") {
        emitFloat("radar_speed_disagreement_gate_mps",
                  config.stage_e_fusion.radar_speed_disagreement_gate_mps);
        continue;
      }
    }

    if (current_section == "front_radar" && key == "output_mode") {
      std::stringstream ss;
      ss << std::string(indent, ' ') << "output_mode: \""
         << normalizeRadarOutputMode(config.front_radar.output_mode) << "\"\n";
      new_content += ss.str();
      continue;
    }

    if (current_section == "mounts" && current_mount == "FrontCam" &&
        key == "xyz_m") {
      auto it = config.mounts.find(Mount::FrontCam);
      if (it != config.mounts.end()) {
        std::stringstream ss;
        ss << std::string(indent, ' ') << "xyz_m: [" << it->second.xyz_m[0]
           << ", " << it->second.xyz_m[1] << ", " << it->second.xyz_m[2]
           << "]\n";
        new_content += ss.str();
        continue;
      }
    }

    new_content += line + "\n";
  }

  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Cannot write config to: " + path);
  }
  out << new_content;
}

HardwareMap ConfigLoader::loadHardwareMap(const std::string &path) {
  HardwareMap map;

  std::string content = readFile(path);

  // Simple JSON parsing for hardware_map.json
  // Expected format: {"mappings": {"FrontCam": "/dev/video0", ...}}
  std::istringstream stream(content);
  std::string line;

  bool inMappings = false;
  while (std::getline(stream, line)) {
    line = trim(line);

    if (line.find("\"schema_version\"") != std::string::npos) {
      size_t start = line.find_last_of(':');
      size_t quote1 = line.find('"', start);
      size_t quote2 = line.find('"', quote1 + 1);
      if (quote1 != std::string::npos && quote2 != std::string::npos) {
        map.schema_version = line.substr(quote1 + 1, quote2 - quote1 - 1);
      }
    } else if (line.find("\"generated_at\"") != std::string::npos) {
      size_t start = line.find_last_of(':');
      size_t quote1 = line.find('"', start);
      size_t quote2 = line.find('"', quote1 + 1);
      if (quote1 != std::string::npos && quote2 != std::string::npos) {
        map.generated_at = line.substr(quote1 + 1, quote2 - quote1 - 1);
      }
    } else if (line.find("\"mappings\"") != std::string::npos) {
      inMappings = true;
    } else if (inMappings) {
      // Parse mount -> path mappings
      // Helper lambda to extract value
      auto extractValue = [&line]() -> std::string {
        size_t colon = line.find(':'); // Find FIRST colon (after key name)
        if (colon == std::string::npos)
          return "";
        size_t quote1 = line.find('"', colon);
        size_t quote2 = line.find('"', quote1 + 1);
        if (quote1 != std::string::npos && quote2 != std::string::npos) {
          return line.substr(quote1 + 1, quote2 - quote1 - 1);
        }
        return "";
      };

      if (line.find("\"FrontCam\"") != std::string::npos) {
        map.mappings[Mount::FrontCam] = extractValue();
      } else if (line.find("\"SideCamL\"") != std::string::npos) {
        map.mappings[Mount::SideCamL] = extractValue();
      } else if (line.find("\"SideCamR\"") != std::string::npos) {
        map.mappings[Mount::SideCamR] = extractValue();
      } else if (line.find("\"RearCam\"") != std::string::npos) {
        map.mappings[Mount::RearCam] = extractValue();
      } else if (line.find("\"FrontRadar\"") != std::string::npos) {
        map.mappings[Mount::FrontRadar] = extractValue();
      } else if (line.find("\"RearCornerRadarL\"") != std::string::npos) {
        map.mappings[Mount::RearCornerRadarL] = extractValue();
      } else if (line.find("\"RearCornerRadarR\"") != std::string::npos) {
        map.mappings[Mount::RearCornerRadarR] = extractValue();
      } else if (line.find("\"IMU\"") != std::string::npos) {
        map.mappings[Mount::IMU] = extractValue();
      } else if (line.find('}') != std::string::npos) {
        inMappings = false;
      }
    }
  }

  return map;
}

bool ConfigLoader::hardwareMapExists(const std::string &path) {
  return std::filesystem::exists(path);
}

void ConfigLoader::saveHardwareMap(const std::string &path,
                                   const HardwareMap &map) {
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot create file: " + path);
  }

  file << "{\n";
  file << "  \"schema_version\": \"" << map.schema_version << "\",\n";
  file << "  \"generated_at\": \"" << map.generated_at << "\",\n";
  file << "  \"mappings\": {\n";

  bool first = true;
  for (const auto &[mount, device_path] : map.mappings) {
    if (!first) {
      file << ",\n";
    }
    file << "    \"" << mountToString(mount) << "\": \"" << device_path << "\"";
    first = false;
  }

  file << "\n  }\n";
  file << "}\n";
}

std::string ConfigLoader::getDefaultConfigPath() {
  return "config/componentConfig.yaml";
}

std::string ConfigLoader::getDefaultHardwareMapPath() {
  return "config/hardware_map.json";
}

} // namespace adas
