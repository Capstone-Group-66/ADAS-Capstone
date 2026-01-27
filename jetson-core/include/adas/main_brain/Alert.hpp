// File: include/adas/main_brain/Alert.hpp
// Alert data structure matching FR70 schema
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace adas {

/**
 * Alert types as defined in proposal Section 4.2.5
 */
enum class AlertType : uint8_t {
    LDW = 0, // Lane Departure Warning
    FCW = 1, // Forward Collision Warning
    RCW = 2, // Rear Collision Warning
    BSD = 3  // Blind Spot Detection
};

/**
 * Severity levels as defined in proposal
 */
enum class Severity : uint8_t { Info = 0, Warning = 1, Critical = 2 };

/**
 * Alert structure matching FR70 schema
 * This is the C++ representation; it will be encoded to CBOR for BLE transmission.
 */
struct Alert {
    uint64_t t_ms;                        // Wallclock timestamp
    std::string id;                       // Unique alert ID (uuid or tick+track)
    AlertType type;                       // LDW, FCW, RCW, BSD
    std::optional<std::string> direction; // "front" or "rear" for CW; omitted for others
    Severity severity;                    // info, warning, critical
    uint32_t ttl_ms;                      // Expiry window relative to t_ms
    std::string rationale;                // JSON string: CW:{ttc_s,...}, LDW:{...}, BSD:{...}
    std::optional<uint32_t> object_id;    // Track ID if applicable
    std::vector<std::string> sources;     // e.g., ["FrontRadar", "FrontCam"]
    std::string schemaVersion;            // "v1.0"
    float confidence;                     // [0.0, 1.0]
};

/**
 * BLE Header (4 bytes) for fragmented transmission
 * Matches struct in proposal Section 4.9.2
 */
struct BLEHeader {
    uint16_t tick_id; // Wraps mod 65536
    uint8_t seq_no;   // Fragment index (0..seq_max)
    uint8_t seq_max;  // Total fragments - 1
};

/**
 * TickPayload wraps alerts for one 20Hz tick
 * This is what gets CBOR-encoded before fragmentation
 */
struct TickPayload {
    uint16_t tick_id;
    uint8_t seq_max;
    uint8_t n; // Number of alerts
    std::vector<Alert> alerts;
};

// Helper to convert AlertType to string
inline const char *alertTypeToString(AlertType t) {
    switch (t) {
    case AlertType::LDW:
        return "LDW";
    case AlertType::FCW:
        return "FCW";
    case AlertType::RCW:
        return "RCW";
    case AlertType::BSD:
        return "BSD";
    default:
        return "UNKNOWN";
    }
}

// Helper to convert Severity to string
inline const char *severityToString(Severity s) {
    switch (s) {
    case Severity::Info:
        return "info";
    case Severity::Warning:
        return "warning";
    case Severity::Critical:
        return "critical";
    default:
        return "unknown";
    }
}

} // namespace adas
