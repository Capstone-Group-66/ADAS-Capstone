// File: include/adas/common/Types.hpp
// Core type definitions for ADAS pipeline
// Per Section 4.2 of proposal documentation
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace adas {

// ══════════════════════════════════════════════════════════════════════════════
//                              MOUNT IDENTITIES
// ══════════════════════════════════════════════════════════════════════════════

/// Mount identities (fixed positions on vehicle)
/// Per D-001: Algorithms bind to Mount, not hardware brand
/// These are POSITIONS, not devices - devices are swappable per FR16
enum class Mount : uint8_t {
    FrontCam = 0,
    SideCamL = 1,
    SideCamR = 2,
    RearCam = 3,
    FrontRadar = 4,
    RearCornerRadarL = 5,
    RearCornerRadarR = 6,
    IMU = 7,
    LiDAR2D = 8,  // Reserved for extensibility (NFR26) — NO IMPLEMENTATION

    COUNT = 9  // For array sizing
};

/// Convert Mount to string for logging/debugging
inline const char* mountToString(Mount m) {
    switch (m) {
        case Mount::FrontCam:
            return "FrontCam";
        case Mount::SideCamL:
            return "SideCamL";
        case Mount::SideCamR:
            return "SideCamR";
        case Mount::RearCam:
            return "RearCam";
        case Mount::FrontRadar:
            return "FrontRadar";
        case Mount::RearCornerRadarL:
            return "RearCornerRadarL";
        case Mount::RearCornerRadarR:
            return "RearCornerRadarR";
        case Mount::IMU:
            return "IMU";
        case Mount::LiDAR2D:
            return "LiDAR2D";
        default:
            return "Unknown";
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//                              SENSOR SOURCE BITMASK
// ══════════════════════════════════════════════════════════════════════════════

/// Sensor source bitmask for tracking which sensors contributed to fusion
/// Used in Track.sources and Alert.sources per FR70 schema
enum SensorSource : uint16_t {
    SRC_NONE = 0,
    SRC_CAM_F = 1 << 0,   // FrontCam
    SRC_CAM_L = 1 << 1,   // SideCamL
    SRC_CAM_R = 1 << 2,   // SideCamR
    SRC_CAM_B = 1 << 3,   // RearCam
    SRC_RAD_F = 1 << 4,   // FrontRadar
    SRC_RAD_L = 1 << 5,   // RearCornerRadarL
    SRC_RAD_R = 1 << 6,   // RearCornerRadarR
    SRC_IMU = 1 << 7,     // IMU
    SRC_LIDAR = 1 << 8    // LiDAR2D - reserved, not implemented
};

/// Combine sensor sources with bitwise OR
inline SensorSource operator|(SensorSource a, SensorSource b) {
    return static_cast<SensorSource>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

inline SensorSource& operator|=(SensorSource& a, SensorSource b) {
    a = a | b;
    return a;
}

// ══════════════════════════════════════════════════════════════════════════════
//                              FRAME/PACKET HEADER
// ══════════════════════════════════════════════════════════════════════════════

/// Authoritative frame/packet header for ALL sensor payloads
/// Per Section 4.2.1 of proposal
struct Header {
    uint64_t t_device_ns;  // Device clock if provided (0 if unavailable)
    uint64_t t_ingest_ns;  // AUTHORITATIVE: receive time (CLOCK_MONOTONIC_RAW)
    Mount mount;           // Which mount this data came from
    uint32_t seq;          // Per-mount sequence counter (monotonic)
    bool healthy;          // Device/ingest health bit

    Header()
        : t_device_ns(0), t_ingest_ns(0), mount(Mount::FrontCam), seq(0), healthy(true) {}

    Header(uint64_t t_ingest, Mount m, uint32_t s, bool h = true)
        : t_device_ns(0), t_ingest_ns(t_ingest), mount(m), seq(s), healthy(h) {}
};

// ══════════════════════════════════════════════════════════════════════════════
//                              PAYLOAD STRUCTURES
// ══════════════════════════════════════════════════════════════════════════════

/// Camera frame payload
/// Raw frame data - undistorting happens in Stage B
/// Contains either raw byte data or cv::Mat (OpenCV)
struct CameraFrameData {
    Header h;
    int width;
    int height;
    int channels;
    std::vector<uint8_t> data;  // BGR pixel data (row-major) - optional
    cv::Mat frame;              // OpenCV Mat for frame data

    CameraFrameData() : width(0), height(0), channels(3) {}
};

/// Single radar target (kept in radar frame until Stage E fusion)
/// Per Section 4.2.2 of proposal
struct RadarTarget {
    float range_m;       // Radial distance to target
    float azimuth_rad;   // Angle in radar frame (+left)
    float radial_vel_mps;  // Toward-negative, away-positive
    float rcs_db;        // Radar cross section (dBsm)
    float sigma_r;       // Measurement stdev for range
    float sigma_az;      // Measurement stdev for azimuth
    float sigma_v;       // Measurement stdev for velocity

    RadarTarget()
        : range_m(0),
          azimuth_rad(0),
          radial_vel_mps(0),
          rcs_db(0),
          sigma_r(0.5f),
          sigma_az(0.05f),
          sigma_v(0.2f) {}
};

/// Batch of radar targets from a single sensor
struct RadarTargets {
    Header h;
    std::vector<RadarTarget> targets;

    RadarTargets() = default;
    explicit RadarTargets(const Header& header) : h(header) {}
};

/// IMU sample (high-rate: ≥100 Hz)
/// Per Section 4.2.2 - sensor frame coordinates
/// Extended for BNO085 data from Pi
struct ImuSample {
    uint64_t t_capture;             // Timestamp (nanoseconds)
    std::array<float, 3> accel;     // Accelerometer [ax, ay, az] m/s²
    std::array<float, 3> gyro;      // Gyroscope [wx, wy, wz] rad/s
    std::array<float, 3> mag;       // Magnetometer [mx, my, mz] µT (optional)
    std::array<float, 4> quat;      // Quaternion [w, x, y, z] (optional)
    float temperature;              // Temperature °C
    uint8_t calibration_status;     // BNO085 calibration 0-3

    ImuSample() 
        : t_capture(0),
          accel{0, 0, 0}, 
          gyro{0, 0, 0}, 
          mag{0, 0, 0},
          quat{1, 0, 0, 0},
          temperature(0),
          calibration_status(0) {}
};

// ══════════════════════════════════════════════════════════════════════════════
//                              NETWORK PROTOCOL (Pi4 → Jetson)
// ══════════════════════════════════════════════════════════════════════════════

/// Network packet types from Raspberry Pi 4
/// Used by NetworkIngest to demultiplex incoming stream
enum class NetPacketType : uint8_t {
    RearCamera = 0x01,    // MJPEG-encoded frame
    RearRadarL = 0x02,    // Left rear corner radar data
    RearRadarR = 0x03,    // Right rear corner radar data
    Heartbeat = 0xFE,     // Keep-alive ping
    Error = 0xFF          // Error indication
};

/// Magic word for packet validation (0xADA5DA7A = "ADAS DATA")
constexpr uint32_t NET_MAGIC_WORD = 0xADA5DA7A;

/// Network packet header (sent by Pi4, received by Jetson)
/// All multi-byte fields are little-endian
#pragma pack(push, 1)
struct NetPacketHeader {
    uint32_t magic;          // Must be NET_MAGIC_WORD
    uint8_t type;            // NetPacketType
    uint8_t reserved;        // Padding for alignment
    uint16_t flags;          // Reserved for future use
    uint32_t payload_size;   // Size of payload in bytes
    uint32_t seq;            // Sequence number (per-type)
    uint64_t pi_timestamp;   // Pi's local timestamp (informational only)
};
#pragma pack(pop)

static_assert(sizeof(NetPacketHeader) == 24, "NetPacketHeader must be 24 bytes");

}  // namespace adas
