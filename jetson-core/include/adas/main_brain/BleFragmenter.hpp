// File: include/adas/main_brain/BleFragmenter.hpp
// BLE packet fragmentation for ADAS alerts
#pragma once

#include <cstdint>
#include <vector>

namespace adas {

/**
 * BLE Fragmentation Constants
 *
 * Based on proposal Section 4.9.2:
 * - Default ATT MTU: 23 bytes
 * - Negotiated MTU: typically 185-512 bytes
 * - Header overhead: 4 bytes (tick_id, seq_no, seq_max)
 * - Slice capacity = MTU - 7 (ATT overhead) - 4 (our header) = MTU - 11
 *
 * For safety, we use conservative defaults that work with any MTU.
 */
constexpr uint16_t DEFAULT_MTU = 23;
constexpr uint16_t BLE_HEADER_SIZE = 4;
constexpr uint16_t ATT_OVERHEAD = 3;  // ATT opcode + handle

/**
 * Calculate slice capacity for a given MTU
 */
inline uint16_t calculateSliceCap(uint16_t mtu) {
    // slice_cap = ATT_MTU - ATT_overhead - BLE_header
    // Conservative: MTU - 7 total overhead
    int16_t cap = static_cast<int16_t>(mtu) - ATT_OVERHEAD - BLE_HEADER_SIZE;
    return (cap > 0) ? static_cast<uint16_t>(cap) : 1;
}

/**
 * Fragment a CBOR payload into BLE frames.
 *
 * Each frame contains:
 *   - Header (4 bytes): tick_id (uint16_t), seq_no (uint8_t), seq_max (uint8_t)
 *   - Slice: portion of the CBOR payload
 *
 * @param tickId     Current tick identifier (wraps at 65536)
 * @param payload    Complete CBOR-encoded payload
 * @param mtu        Current negotiated MTU
 * @return           Vector of framed packets ready for BLE notification
 */
std::vector<std::vector<uint8_t>> fragmentPayload(
    uint16_t tickId,
    const std::vector<uint8_t>& payload,
    uint16_t mtu = DEFAULT_MTU);

/**
 * Create a single BLE frame with header + slice
 */
std::vector<uint8_t> createFrame(
    uint16_t tickId,
    uint8_t seqNo,
    uint8_t seqMax,
    const uint8_t* sliceData,
    size_t sliceLen);

}  // namespace adas
