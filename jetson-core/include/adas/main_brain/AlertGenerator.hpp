// File: include/adas/main_brain/AlertGenerator.hpp
#pragma once

#include "adas/main_brain/Alert.hpp"
#include <cstdint>
#include <vector>

namespace adas {

// Generate a test alert for demo purposes
Alert generateTestAlert(AlertType type, uint32_t sequenceNum);

// Encode an Alert to CBOR bytes (proper binary CBOR using nlohmann/json)
std::vector<uint8_t> encodeAlertToCbor(const Alert &alert);

// Encode a full TickPayload to CBOR bytes (compact format for mobile app)
std::vector<uint8_t> encodeTickPayloadToCbor(uint16_t tickId, int speedKmh, int healthMask,
                                             int bsdMask, const std::vector<Alert> &alerts);

// Legacy overload (defaults speed=0, health=0, bsd=0)
std::vector<uint8_t> encodeTickPayloadToCbor(uint16_t tickId, const std::vector<Alert> &alerts);

// Create a framed BLE packet: Header (4 bytes) + Payload slice
std::vector<uint8_t> createBleFrame(uint16_t tickId, uint8_t seqNo, uint8_t seqMax,
                                    const std::vector<uint8_t> &slice);

} // namespace adas
