// File: include/adas/common/DeepStreamIPC.hpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace adas {
namespace ipc {

// ─────────────────────────────────────────────────────────────────────────────
// DEEPSTREAM DETECTIONS (IPC from local DeepStream C application)
// ─────────────────────────────────────────────────────────────────────────────
#pragma pack(push, 1)
struct DeepStreamDetBatchHeader {
    uint64_t timestamp_ns;   ///< CLOCK_MONOTONIC timestamp of the frame
    uint32_t num_detections; ///< Number of DeepStreamDet structs following
    uint32_t reserved;       ///< Padding for 8-byte alignment
};
#pragma pack(pop)
static_assert(sizeof(DeepStreamDetBatchHeader) == 16, "DeepStreamDetBatchHeader must be 16 bytes");

#pragma pack(push, 1)
struct DeepStreamDet {
    float x;
    float y;
    float w;
    float h;
    float centroid_x;
    float centroid_y;
    int32_t cls;
    float score;
    uint64_t object_id;
    char sign_type[32]; // Must match DeepStream sender layout exactly.
};
#pragma pack(pop)
static_assert(offsetof(DeepStreamDet, sign_type) == 40,
              "DeepStreamDet::sign_type offset must be 40 bytes");
static_assert(sizeof(DeepStreamDet) == 72, "DeepStreamDet must be 72 bytes");

} // namespace ipc
} // namespace adas
