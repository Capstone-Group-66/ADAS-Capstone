// File: src/main_brain/BleFragmenter.cpp
// BLE packet fragmentation implementation
#include "adas/main_brain/BleFragmenter.hpp"

#include <algorithm>
#include <cmath>

namespace adas {

std::vector<uint8_t> createFrame(
    uint16_t tickId,
    uint8_t seqNo,
    uint8_t seqMax,
    const uint8_t* sliceData,
    size_t sliceLen) {

    std::vector<uint8_t> frame;
    frame.reserve(BLE_HEADER_SIZE + sliceLen);

    // Header (4 bytes, little-endian)
    frame.push_back(static_cast<uint8_t>(tickId & 0xFF));
    frame.push_back(static_cast<uint8_t>((tickId >> 8) & 0xFF));
    frame.push_back(seqNo);
    frame.push_back(seqMax);

    // Payload slice
    frame.insert(frame.end(), sliceData, sliceData + sliceLen);

    return frame;
}

std::vector<std::vector<uint8_t>> fragmentPayload(
    uint16_t tickId,
    const std::vector<uint8_t>& payload,
    uint16_t mtu) {

    std::vector<std::vector<uint8_t>> frames;

    // Calculate slice capacity
    uint16_t sliceCap = calculateSliceCap(mtu);

    // Calculate number of fragments needed
    size_t payloadSize = payload.size();
    size_t numFragments = (payloadSize + sliceCap - 1) / sliceCap;  // Ceiling division

    // Clamp to max 256 fragments (seq_no and seq_max are uint8_t)
    if (numFragments > 256) {
        numFragments = 256;
    }
    if (numFragments == 0) {
        numFragments = 1;  // At least one empty frame
    }

    uint8_t seqMax = static_cast<uint8_t>(numFragments - 1);

    frames.reserve(numFragments);

    for (size_t i = 0; i < numFragments; ++i) {
        size_t offset = i * sliceCap;
        size_t remaining = payloadSize - offset;
        size_t sliceLen = std::min(static_cast<size_t>(sliceCap), remaining);

        const uint8_t* sliceData = payload.data() + offset;

        frames.push_back(createFrame(
            tickId,
            static_cast<uint8_t>(i),  // seq_no
            seqMax,
            sliceData,
            sliceLen
        ));
    }

    return frames;
}

}  // namespace adas
