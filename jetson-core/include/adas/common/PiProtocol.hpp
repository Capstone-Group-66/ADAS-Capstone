// File: include/adas/common/PiProtocol.hpp
// Shared protocol definitions for Pi4 ↔ Jetson communication
// This file should be copied to the Pi codebase for consistency
#pragma once

#include <cstdint>

namespace adas {
namespace protocol {

// ═══════════════════════════════════════════════════════════════════════════
//                              CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint32_t PI_MAGIC = 0x50493034;  // "PI04" in little-endian
constexpr uint16_t PI_PROTOCOL_VERSION = 0x0100;  // v1.0

// Port assignments
constexpr int PORT_REAR_CAM = 5555;
constexpr int PORT_RADAR_L = 5556;
constexpr int PORT_RADAR_R = 5557;
constexpr int PORT_IMU = 5558;
constexpr int PORT_CONTROL = 5559;

// ═══════════════════════════════════════════════════════════════════════════
//                              MESSAGE TYPES
// ═══════════════════════════════════════════════════════════════════════════

enum class MessageType : uint16_t {
    // Sensor data (0x00XX)
    REAR_CAM_FRAME  = 0x0001,
    REAR_RADAR_L    = 0x0002,
    REAR_RADAR_R    = 0x0003,
    IMU_SAMPLE      = 0x0004,
    
    // Control (0x00XX)
    HEARTBEAT       = 0x0010,
    
    // Discovery (0x002X)
    DISCOVERY_REQ   = 0x0020,
    DISCOVERY_RSP   = 0x0021,
};

// ═══════════════════════════════════════════════════════════════════════════
//                              MESSAGE HEADER
// ═══════════════════════════════════════════════════════════════════════════

// All messages start with this 32-byte header
#pragma pack(push, 1)
struct PiMessageHeader {
    uint32_t magic;           // Must be PI_MAGIC (0x50493034)
    uint16_t version;         // Protocol version (PI_PROTOCOL_VERSION)
    uint16_t msg_type;        // MessageType enum value
    uint32_t payload_size;    // Size of payload in bytes (after header)
    uint64_t timestamp_ns;    // Unix epoch nanoseconds (Chrony-synced!)
    uint32_t sequence;        // Per-stream sequence number (for drop detection)
    uint32_t reserved;        // Set to 0
};
#pragma pack(pop)

static_assert(sizeof(PiMessageHeader) == 32, "Header must be exactly 32 bytes");

// ═══════════════════════════════════════════════════════════════════════════
//                              PAYLOAD STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════

// Camera frame encoding types
enum class CameraEncoding : uint8_t {
    RAW_BGR = 0,
    MJPEG = 1,
    H264 = 2,
};

// Camera payload header (followed by encoded frame data)
#pragma pack(push, 1)
struct CameraPayloadHeader {
    uint16_t width;           // Frame width in pixels
    uint16_t height;          // Frame height in pixels
    uint8_t  encoding;        // CameraEncoding enum
    uint8_t  reserved[3];     // Padding to 8 bytes
    // Followed by: uint8_t data[payload_size - 8]
};
#pragma pack(pop)

static_assert(sizeof(CameraPayloadHeader) == 8, "CameraPayloadHeader must be 8 bytes");

// Radar payload header (followed by raw serial data)
#pragma pack(push, 1)
struct RadarPayloadHeader {
    uint16_t data_length;     // Length of raw serial data
    uint8_t  radar_type;      // 0=OPS243, 1=other
    uint8_t  reserved;        // Padding
    // Followed by: uint8_t raw_data[data_length]
};
#pragma pack(pop)

static_assert(sizeof(RadarPayloadHeader) == 4, "RadarPayloadHeader must be 4 bytes");

// IMU sample data
#pragma pack(push, 1)
struct ImuPayload {
    // Accelerometer (m/s²)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // Gyroscope (rad/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // Magnetometer (µT) - optional, set to 0 if unavailable
    float mag_x;
    float mag_y;
    float mag_z;
    
    // Quaternion orientation (if sensor provides it)
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    
    // Temperature (°C)
    float temperature;
    
    // Status
    uint8_t  calibration_status;  // 0-3 per BNO085 spec
    uint8_t  reserved[3];
};
#pragma pack(pop)

static_assert(sizeof(ImuPayload) == 60, "ImuPayload must be 60 bytes");

// Heartbeat payload
#pragma pack(push, 1)
struct HeartbeatPayload {
    uint64_t uptime_ms;           // Pi uptime in milliseconds
    uint8_t  rear_cam_healthy;    // 1=OK, 0=ERROR
    uint8_t  radar_l_healthy;     // 1=OK, 0=ERROR
    uint8_t  radar_r_healthy;     // 1=OK, 0=ERROR
    uint8_t  imu_healthy;         // 1=OK, 0=ERROR
    int32_t  chrony_offset_us;    // Clock offset to Jetson (microseconds)
    uint32_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(HeartbeatPayload) == 20, "HeartbeatPayload must be 20 bytes");

// ═══════════════════════════════════════════════════════════════════════════
//                              DISCOVERY
// ═══════════════════════════════════════════════════════════════════════════

// Mount identifiers (must match adas::Mount enum)
enum class MountId : uint8_t {
    FRONT_CAM       = 0,
    SIDE_CAM_L      = 1,
    SIDE_CAM_R      = 2,
    REAR_CAM        = 3,   // Pi hosts this
    FRONT_RADAR     = 4,
    REAR_RADAR_L    = 5,   // Pi hosts this
    REAR_RADAR_R    = 6,   // Pi hosts this
    IMU             = 7,   // Pi hosts this
};

// Device types
enum class DeviceType : uint8_t {
    CAMERA  = 1,
    RADAR   = 2,
    IMU     = 3,
};

// Device status
enum class DeviceStatus : uint16_t {
    OK              = 0,
    ERROR           = 1,
    NOT_CONNECTED   = 2,
};

// Single device info in discovery response
#pragma pack(push, 1)
struct DeviceInfo {
    uint8_t  device_type;     // DeviceType enum
    uint8_t  mount_id;        // MountId enum
    uint16_t status;          // DeviceStatus enum
    char     serial[16];      // Null-terminated serial number
};
#pragma pack(pop)

static_assert(sizeof(DeviceInfo) == 20, "DeviceInfo must be 20 bytes");

// Discovery response payload
#pragma pack(push, 1)
struct DiscoveryResponsePayload {
    uint8_t  num_devices;     // Number of DeviceInfo entries following
    uint8_t  reserved[3];
    // Followed by: DeviceInfo devices[num_devices]
};
#pragma pack(pop)

// ═══════════════════════════════════════════════════════════════════════════
//                              HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

inline bool validateHeader(const PiMessageHeader& header) {
    return header.magic == PI_MAGIC && 
           header.version == PI_PROTOCOL_VERSION;
}

// Pi-hosted devices (always present on Pi)
inline constexpr MountId PI_HOSTED_DEVICES[] = {
    MountId::REAR_CAM,
    MountId::REAR_RADAR_L,
    MountId::REAR_RADAR_R,
    MountId::IMU,  // IMU is always on Pi
};

inline constexpr size_t NUM_PI_DEVICES = sizeof(PI_HOSTED_DEVICES) / sizeof(PI_HOSTED_DEVICES[0]);

} // namespace protocol
} // namespace adas
