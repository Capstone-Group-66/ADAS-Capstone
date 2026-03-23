// File: src/main_brain/AlertGenerator.cpp
// Generates test alerts and encodes them to CBOR for BLE transmission
#include "adas/main_brain/AlertGenerator.hpp"

#include "adas/main_brain/Alert.hpp"
#include "adas/main_brain/BleUuids.hpp"

// Using nlohmann/json for CBOR encoding (single-header library)
// Include the single-header version from the vendor directory
#include <chrono>
#include <cstdint>
#include <vector>

#include "nlohmann/json.hpp"

namespace adas {

using json = nlohmann::json;

/**
 * Generate a test alert for demo purposes.
 * In the full implementation, alerts come from Stage F (Decision Logic).
 */
Alert generateTestAlert(AlertType type, uint32_t sequenceNum) {
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();

  Alert alert;
  alert.t_ms = static_cast<uint64_t>(ms);

  // Generate unique ID
  std::ostringstream idss;
  idss << "alert_" << sequenceNum << "_" << (ms % 100000);
  alert.id = idss.str();

  alert.type = type;
  alert.severity = Severity::Warning;
  alert.ttl_ms = 2000; // 2 second TTL
  alert.schemaVersion = "v1.0";
  alert.confidence = 0.85f;

  // Type-specific fields
  switch (type) {
  case AlertType::LDW:
    alert.rationale = R"({"d_ego_lane_m": 0.35, "drift_dir": "left"})";
    alert.sources = {"FrontCam"};
    break;

  case AlertType::FCW:
    alert.direction = "front";
    alert.rationale = R"({"ttc_s": 1.8, "in_path": true, "closing": true})";
    alert.object_id = 42;
    alert.sources = {"FrontCam", "FrontRadar"};
    alert.severity = Severity::Critical;
    break;

  case AlertType::RCW:
    alert.direction = "rear";
    alert.rationale = R"({"alert": 1, "status": 2})";
    alert.sources = {"RearRCW"};
    break;

  case AlertType::BSD:
    alert.rationale = R"({"zone": "L", "entering": true})";
    alert.sources = {"RearCornerRadarL"};
    break;
  }

  return alert;
}

/**
 * Convert Alert to JSON object (for CBOR serialization)
 */
json alertToJson(const Alert &alert) {
  json j;
  j["t_ms"] = alert.t_ms;
  j["id"] = alert.id;
  j["type"] = alertTypeToString(alert.type);
  j["severity"] = severityToString(alert.severity);
  j["ttl_ms"] = alert.ttl_ms;
  j["rationale"] = json::parse(alert.rationale); // Parse nested JSON
  j["sources"] = alert.sources;
  j["schemaVersion"] = alert.schemaVersion;
  j["confidence"] = alert.confidence;

  if (alert.direction) {
    j["direction"] = *alert.direction;
  }
  if (alert.object_id) {
    j["object_id"] = *alert.object_id;
  }

  return j;
}

/**
 * Encode an Alert to CBOR bytes using nlohmann/json.
 * This produces the exact binary format that the mobile app expects.
 */
std::vector<uint8_t> encodeAlertToCbor(const Alert &alert) {
  json j = alertToJson(alert);
  return json::to_cbor(j);
}

/**
 * Convert AlertType enum to wire value for mobile app.
 * Wire format: 0=FCW, 1=LDW, 2=RCW, 3=BSD
 */
int alertTypeToWireValue(AlertType type) {
  switch (type) {
  case AlertType::FCW:
    return 0;
  case AlertType::LDW:
    return 1;
  case AlertType::RCW:
    return 2;
  case AlertType::BSD:
    return 3;
  default:
    return 0;
  }
}

/**
 * Convert Alert to short-key JSON for mobile app CBOR format.
 * Uses compact keys: id, s, r
 */
json alertToCompactJson(const Alert &alert) {
  json j;
  j["id"] = alertTypeToWireValue(alert.type);
  j["s"] = static_cast<int>(alert.severity);
  j["r"] = alert.rationale; // Keep as string, mobile parses if needed
  return j;
}

/**
 * Encode a TickPayload to CBOR bytes using the compact format.
 * Uses short keys: t, v, h, b, a matching TickPayload.kt
 *
 * @param tickId Tick counter (wraps at 65536)
 * @param speedKmh Current vehicle speed in km/h
 * @param healthMask Sensor health bitmask (application-defined compact flags)
 * @param bsdMask BSD zone bitmask (bit0=left, bit1=right)
 * @param alerts Vector of alerts to include
 */
std::vector<uint8_t> encodeTickPayloadToCbor(uint16_t tickId, int speedKmh,
                                             int healthMask, int bsdMask,
                                             const std::vector<Alert> &alerts) {
  json payload;
  payload["t"] = tickId;
  payload["v"] = speedKmh;
  payload["h"] = healthMask;
  payload["b"] = bsdMask;

  json alertsArray = json::array();
  for (const auto &alert : alerts) {
    alertsArray.push_back(alertToCompactJson(alert));
  }
  payload["a"] = alertsArray;

  return json::to_cbor(payload);
}

// Legacy overload for backwards compatibility (defaults speed=0, health=0,
// bsd=0)
std::vector<uint8_t> encodeTickPayloadToCbor(uint16_t tickId,
                                             const std::vector<Alert> &alerts) {
  return encodeTickPayloadToCbor(tickId, 0, 0, 0, alerts);
}

/**
 * Create a framed BLE packet with Header + Payload
 * This matches the BLE Alert Framing spec in proposal Section 4.9
 */
std::vector<uint8_t> createBleFrame(uint16_t tickId, uint8_t seqNo,
                                    uint8_t seqMax,
                                    const std::vector<uint8_t> &slice) {
  std::vector<uint8_t> frame;
  frame.reserve(4 + slice.size());

  // Header (4 bytes, little-endian)
  frame.push_back(static_cast<uint8_t>(tickId & 0xFF));
  frame.push_back(static_cast<uint8_t>((tickId >> 8) & 0xFF));
  frame.push_back(seqNo);
  frame.push_back(seqMax);

  // Payload slice
  frame.insert(frame.end(), slice.begin(), slice.end());

  return frame;
}

} // namespace adas
