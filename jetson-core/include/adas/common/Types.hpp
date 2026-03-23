// File: include/adas/common/Types.hpp
// Core type definitions for ADAS pipeline
// Per Section 4.2 of proposal documentation
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
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
    SideCamL = 1, // Reserved legacy mount (no active runtime path)
    SideCamR = 2, // Reserved legacy mount (no active runtime path)
    RearCam = 3,  // Reserved legacy mount (used only for compatibility)
    FrontRadar = 4,
    RearCornerRadarL = 5,
    RearCornerRadarR = 6,
    IMU = 7,
    LiDAR2D = 8, // Reserved for extensibility (NFR26) — NO IMPLEMENTATION

    COUNT = 9 // For array sizing
};

/// Convert Mount to string for logging/debugging
inline const char *mountToString(Mount m) {
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
    SRC_CAM_F = 1 << 0, // FrontCam
    SRC_CAM_L = 1 << 1, // Reserved legacy side-camera bit
    SRC_CAM_R = 1 << 2, // Reserved legacy side-camera bit
    SRC_CAM_B = 1 << 3, // Reserved legacy rear-camera bit
    SRC_RAD_F = 1 << 4, // FrontRadar
    SRC_RAD_L = 1 << 5, // RearCornerRadarL
    SRC_RAD_R = 1 << 6, // RearCornerRadarR
    SRC_IMU = 1 << 7,   // IMU
    SRC_LIDAR = 1 << 8  // LiDAR2D - reserved, not implemented
};

/// Combine sensor sources with bitwise OR
inline SensorSource operator|(SensorSource a, SensorSource b) {
    return static_cast<SensorSource>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

inline SensorSource &operator|=(SensorSource &a, SensorSource b) {
    a = a | b;
    return a;
}

// ══════════════════════════════════════════════════════════════════════════════
//                              FRAME/PACKET HEADER
// ══════════════════════════════════════════════════════════════════════════════

/// Authoritative frame/packet header for ALL sensor payloads
/// Per Section 4.2.1 of proposal
struct Header {
    uint64_t t_device_ns; // Device clock if provided (0 if unavailable)
    uint64_t t_ingest_ns; // AUTHORITATIVE: receive time (CLOCK_MONOTONIC_RAW)
    Mount mount;          // Which mount this data came from
    uint32_t seq;         // Per-mount sequence counter (monotonic)
    bool healthy;         // Device/ingest health bit

    Header() : t_device_ns(0), t_ingest_ns(0), mount(Mount::FrontCam), seq(0), healthy(true) {}

    Header(uint64_t t_ingest, Mount m, uint32_t s, bool h = true)
        : t_device_ns(0), t_ingest_ns(t_ingest), mount(m), seq(s), healthy(h) {}
};

// ══════════════════════════════════════════════════════════════════════════════
//                              PAYLOAD STRUCTURES
// ══════════════════════════════════════════════════════════════════════════════

/// Camera frame payload
/// Raw frame data from camera ingest paths
/// Contains either raw byte data or cv::Mat (OpenCV)
struct CameraFrameData {
    Header h;
    int width;
    int height;
    int channels;
    std::vector<uint8_t> data; // BGR pixel data (row-major) - optional
    cv::Mat frame;             // OpenCV Mat for frame data

    CameraFrameData() : width(0), height(0), channels(3) {}
};

/// Single radar target (kept in radar frame until Stage E fusion)
/// Per Section 4.2.2 of proposal
struct RadarTarget {
    float range_m;        // Radial distance to target
    float azimuth_rad;    // Angle in radar frame (+left)
    float radial_vel_mps; // Toward/inward-positive, away/outward-negative
    float rcs_db;         // Radar cross section (dBsm)
    float sigma_r;        // Measurement stdev for range
    float sigma_az;       // Measurement stdev for azimuth
    float sigma_v;        // Measurement stdev for velocity
    
    // Fusion metadata
    bool is_fused;        // True if part of a fused frame
    bool speed_fresh;     // True if speed within TTL
    uint32_t speed_age_ms;// Age of speed measurement

    RadarTarget()
        : range_m(0), azimuth_rad(0), radial_vel_mps(0), rcs_db(0), sigma_r(0.5f), sigma_az(0.05f),
          sigma_v(0.2f), is_fused(false), speed_fresh(false), speed_age_ms(0) {}
};

/// Batch of radar targets from a single sensor
struct RadarTargets {
    Header h;
    std::vector<RadarTarget> targets;

    RadarTargets() = default;
    explicit RadarTargets(const Header &header) : h(header) {}
};

/// IMU sample (high-rate: ≥100 Hz)
/// Per Section 4.2.2 - sensor frame coordinates
/// Extended for BNO085 data from Pi
struct ImuSample {
    uint64_t t_capture;         // Timestamp (nanoseconds)
    std::array<float, 3> accel; // Accelerometer [ax, ay, az] m/s²
    std::array<float, 3> gyro;  // Gyroscope [wx, wy, wz] rad/s
    std::array<float, 3> mag;   // Magnetometer [mx, my, mz] µT (optional)
    std::array<float, 4> quat;  // Quaternion [w, x, y, z] (optional)
    float temperature;          // Temperature °C
    uint8_t calibration_status; // BNO085 calibration 0-3

    ImuSample()
        : t_capture(0), accel{0, 0, 0}, gyro{0, 0, 0}, mag{0, 0, 0}, quat{1, 0, 0, 0},
          temperature(0), calibration_status(0) {}
};

/// Compact rear-collision state received from the Pi over ZMQ port 5555.
/// `alert != 0` means RCW is active. `status` is preserved verbatim from the
/// Pi publisher for logging/replay/BLE rationale.
struct RcwState {
    Header h;
    uint8_t alert;
    uint8_t status;

    RcwState() : h(), alert(0), status(0) {
        h.mount = Mount::RearCam; // Reserved compatibility provenance slot.
    }
};

// ══════════════════════════════════════════════════════════════════════════════
//                              DETECTION STRUCTURES
// ══════════════════════════════════════════════════════════════════════════════

/// Canonical front-camera class IDs used inside jetson-core.
/// These follow the active DeepStream / DashCamNet schema so Stage E, replay,
/// and debug tooling all interpret classes the same way:
///   0 = Car
///   1 = Bicycle
///   2 = Person
///   3 = RoadSign
///
/// Older COCO-style paths must be explicitly bridged into this schema before
/// populating Det.cls.
enum class ObjectClass : uint8_t {
    Car = 0,
    Bicycle = 1,
    Person = 2,
    RoadSign = 3,
    Unknown = 255
};

/// Convert a DeepStream / DashCamNet class ID into the canonical front-camera
/// schema. Unknown / out-of-range values degrade to Unknown safely.
inline ObjectClass deepStreamToObjectClass(int ds_id) {
    switch (ds_id) {
    case 0:
        return ObjectClass::Car;
    case 1:
        return ObjectClass::Bicycle;
    case 2:
        return ObjectClass::Person;
    case 3:
        return ObjectClass::RoadSign;
    default:
        return ObjectClass::Unknown;
    }
}

/// Convert a legacy COCO-style detector class ID into the canonical
/// DeepStream-aligned schema used by the rest of jetson-core.
///
/// Vehicle-like COCO classes (car, motorcycle, bus, truck) are intentionally
/// collapsed to the canonical `Car` bucket because the active DeepStream path
/// exposes only one forward-vehicle class.
inline ObjectClass cocoToObjectClass(int coco_id) {
    switch (coco_id) {
    case 2:
        return ObjectClass::Car;
    case 1:
        return ObjectClass::Bicycle;
    case 0:
        return ObjectClass::Person;
    case 3:
    case 5:
    case 7:
        return ObjectClass::Car;
    default:
        return ObjectClass::Unknown;
    }
}

inline const char *objectClassToString(int class_id) {
    switch (static_cast<ObjectClass>(class_id)) {
    case ObjectClass::Car:
        return "Car";
    case ObjectClass::Bicycle:
        return "Bicycle";
    case ObjectClass::Person:
        return "Person";
    case ObjectClass::RoadSign:
        return "RoadSign";
    default:
        return "Unknown";
    }
}

/// Single detection from camera
/// Per Section 4.2.2 of proposal
struct Det {
    cv::Rect2f  box_px;     // [x, y, w, h] in pixels (after preproc)
    cv::Point2f centroid;   // Center point in pixels (for fusion)
    int         cls;        // Class ID (canonical ObjectClass enum value)
    float       score;      // Confidence score [0, 1]
    uint64_t    object_id;  // Persistent tracker ID from nvtracker (DeepStream).
                            // UINT64_MAX = untracked (YOLO/non-DS path).
                            // Used by Stage E radar-camera fusion to match
                            // the same physical object across frames.
    std::array<char, 32> sign_label; // Optional DeepStream SGIE road-sign label.

    Det()
        : box_px(), centroid(),
          cls(static_cast<int>(ObjectClass::Unknown)),
          score(0.0f),
          object_id(UINT64_MAX),
          sign_label{} {}

    Det(const cv::Rect2f& box, int class_id, float confidence,
        uint64_t track_id = UINT64_MAX, const char *sign_text = nullptr)
        : box_px(box),
          centroid(box.x + box.width / 2.0f, box.y + box.height / 2.0f),
          cls(class_id),
          score(confidence),
          object_id(track_id),
          sign_label{} {
        setSignLabel(sign_text);
    }

    void setSignLabel(const char *label) {
        sign_label.fill('\0');
        if (!label) {
            return;
        }
        std::strncpy(sign_label.data(), label, sign_label.size() - 1);
        sign_label[sign_label.size() - 1] = '\0';
    }

    bool hasSignLabel() const { return sign_label[0] != '\0'; }

    std::string signLabelString() const { return std::string(sign_label.data()); }
};

/// Batch of detections from a single frame
/// Per Section 4.2.2 of proposal
struct DetBatch {
    Header h;                   // Inherited from source frame
    std::vector<Det> dets;      // All detections in frame
    uint64_t inference_time_us; // Inference latency (microseconds)
    cv::Mat frame;              // Undistorted frame for visualization (optional)

    DetBatch() : inference_time_us(0) {}
};

// ══════════════════════════════════════════════════════════════════════════════
//                              NETWORK PROTOCOL (Pi4 → Jetson)
// ══════════════════════════════════════════════════════════════════════════════

/// Legacy TCP packet types from Raspberry Pi 4.
/// Kept only for compatibility with deprecated tooling.
enum class NetPacketType : uint8_t {
    RearCamera = 0x01, // Legacy rear-video packet (deprecated)
    RearRadarL = 0x02, // Left rear corner radar data
    RearRadarR = 0x03, // Right rear corner radar data
    Heartbeat = 0xFE,  // Keep-alive ping
    Error = 0xFF       // Error indication
};

/// Magic word for packet validation (0xADA5DA7A = "ADAS DATA")
constexpr uint32_t NET_MAGIC_WORD = 0xADA5DA7A;

/// Network packet header (sent by Pi4, received by Jetson)
/// All multi-byte fields are little-endian
#pragma pack(push, 1)
struct NetPacketHeader {
    uint32_t magic;        // Must be NET_MAGIC_WORD
    uint8_t type;          // NetPacketType
    uint8_t reserved;      // Padding for alignment
    uint16_t flags;        // Reserved for future use
    uint32_t payload_size; // Size of payload in bytes
    uint32_t seq;          // Sequence number (per-type)
    uint64_t pi_timestamp; // Pi's local timestamp (informational only)
};
#pragma pack(pop)

static_assert(sizeof(NetPacketHeader) == 24, "NetPacketHeader must be 24 bytes");

} // namespace adas

