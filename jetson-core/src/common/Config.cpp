// File: src/common/Config.cpp
// Configuration loader implementation
#include "adas/common/Config.hpp"

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

} // namespace

namespace adas {

Config ConfigLoader::loadConfig(const std::string &path) {
    Config config;

    std::string content = readFile(path);

    // Simple YAML parsing for our known structure
    // In production, use yaml-cpp library
    std::istringstream stream(content);
    std::string line;

    while (std::getline(stream, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Parse key-value pairs
        size_t colonPos = line.find(':');
        if (colonPos == std::string::npos) {
            continue;
        }

        std::string key = trim(line.substr(0, colonPos));
        std::string value = trim(line.substr(colonPos + 1));

        // Time config
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

        // Camera config
        else if (key == "width") {
            config.cameras.width = std::stoi(value);
        } else if (key == "height") {
            config.cameras.height = std::stoi(value);
        } else if (key == "target_fps") {
            config.cameras.target_fps = std::stoi(value);
        } else if (key == "use_mjpeg") {
            config.cameras.use_mjpeg = (value == "true");
        }

        // Network config
        else if (key == "port") {
            config.network.port = std::stoi(value);
        } else if (key == "latency_correction_ms") {
            config.network.latency_correction_ms = std::stoi(value);
        } else if (key == "reconnect_timeout_ms") {
            config.network.reconnect_timeout_ms = std::stoi(value);
        }

        // Radar config
        else if (key == "baud_rate") {
            config.front_radar.baud_rate = std::stoi(value);
        } else if (key == "poll_timeout_ms") {
            config.front_radar.poll_timeout_ms = std::stoi(value);
        }

        // IMU config
        else if (key == "rate_hz") {
            config.imu.rate_hz = std::stoi(value);
        } else if (key == "use_uart") {
            config.imu.use_uart = (value == "true");
        }
    }

    return config;
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
            if (line.find("\"FrontCam\"") != std::string::npos) {
                size_t start = line.find_last_of(':');
                size_t quote1 = line.find('"', start);
                size_t quote2 = line.find('"', quote1 + 1);
                if (quote1 != std::string::npos && quote2 != std::string::npos) {
                    map.mappings[Mount::FrontCam] =
                        line.substr(quote1 + 1, quote2 - quote1 - 1);
                }
            } else if (line.find("\"SideCamL\"") != std::string::npos) {
                size_t start = line.find_last_of(':');
                size_t quote1 = line.find('"', start);
                size_t quote2 = line.find('"', quote1 + 1);
                if (quote1 != std::string::npos && quote2 != std::string::npos) {
                    map.mappings[Mount::SideCamL] =
                        line.substr(quote1 + 1, quote2 - quote1 - 1);
                }
            } else if (line.find("\"SideCamR\"") != std::string::npos) {
                size_t start = line.find_last_of(':');
                size_t quote1 = line.find('"', start);
                size_t quote2 = line.find('"', quote1 + 1);
                if (quote1 != std::string::npos && quote2 != std::string::npos) {
                    map.mappings[Mount::SideCamR] =
                        line.substr(quote1 + 1, quote2 - quote1 - 1);
                }
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
