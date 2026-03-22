// File: include/adas/recording/Recorder.hpp
// Record sensor data to .adasrec binary file for offline replay
#pragma once

#include "adas/common/Clock.hpp"
#include "adas/common/Types.hpp"

#include <atomic>
#include <cstdint>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace adas {

// ═══════════════════════════════════════════════════════════════════════════════
//                        .adasrec FILE FORMAT (v2 current)
// ═══════════════════════════════════════════════════════════════════════════════

/// File header: 32 bytes
struct AdasRecFileHeader {
    char magic[4] = {'A', 'R', 'E', 'C'};
    uint16_t version = 2;
    uint16_t reserved1 = 0;
    uint64_t start_time_ns = 0;
    uint16_t sensor_mask = 0;
    uint8_t reserved2[14] = {};
};
static_assert(sizeof(AdasRecFileHeader) == 32, "File header must be 32 bytes");

/// Event types — sensor stream IDs
enum class RecEventType : uint8_t {
    CameraFront  = 0x01,
    CameraSideL  = 0x02,
    CameraSideR  = 0x03,
    CameraRear   = 0x04,
    RadarFront   = 0x10,
    RadarRearL   = 0x11,
    RadarRearR   = 0x12,
    IMU          = 0x20,
    GPS          = 0x30,
    FrontDetBatch = 0x40,
};

/// Event header: 13 bytes (packed)
#pragma pack(push, 1)
struct RecEventHeader {
    uint64_t timestamp_ns;
    uint8_t type;
    uint32_t payload_size;
};
#pragma pack(pop)
static_assert(sizeof(RecEventHeader) == 13, "Event header must be 13 bytes");

#pragma pack(push, 1)
struct RecFrontDetBatchHeader {
    uint32_t num_detections = 0;
    uint32_t reserved = 0;
    uint64_t inference_time_us = 0;
};
#pragma pack(pop)
static_assert(sizeof(RecFrontDetBatchHeader) == 16,
              "Front detection batch header must be 16 bytes");

#pragma pack(push, 1)
struct RecFrontDet {
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
    float centroid_x = 0.0f;
    float centroid_y = 0.0f;
    int32_t cls = 0;
    float score = 0.0f;
    uint64_t object_id = UINT64_MAX;
    char sign_label[32] = {};
};
#pragma pack(pop)
static_assert(sizeof(RecFrontDet) == 72,
              "Front detection record must be 72 bytes");

/// Convert Mount enum to RecEventType for cameras
inline RecEventType mountToCameraEvent(Mount m) {
    switch (m) {
    case Mount::FrontCam:  return RecEventType::CameraFront;
    case Mount::SideCamL:  return RecEventType::CameraSideL;
    case Mount::SideCamR:  return RecEventType::CameraSideR;
    case Mount::RearCam:   return RecEventType::CameraRear;
    default:               return RecEventType::CameraFront;
    }
}

/// Convert Mount enum to RecEventType for radars
inline RecEventType mountToRadarEvent(Mount m) {
    switch (m) {
    case Mount::FrontRadar:       return RecEventType::RadarFront;
    case Mount::RearCornerRadarL: return RecEventType::RadarRearL;
    case Mount::RearCornerRadarR: return RecEventType::RadarRearR;
    default:                      return RecEventType::RadarFront;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//                              RECORDER CLASS
// ═══════════════════════════════════════════════════════════════════════════════

/// Internal event for the writer queue
struct RecordEvent {
    RecEventType type;
    uint64_t timestamp_ns;
    std::vector<uint8_t> payload;
};

/// Recorder: captures all sensor data to a .adasrec file.
/// Thread-safe: record*() methods are called from producer threads,
/// data is buffered and written by a dedicated writer thread.
class Recorder {
  public:
    Recorder() = default;
    ~Recorder();

    // Non-copyable
    Recorder(const Recorder &) = delete;
    Recorder &operator=(const Recorder &) = delete;

    /// Start recording to a file in the given directory.
    /// Creates: <output_dir>/run_YYYYMMDD_HHMMSS.adasrec
    /// @return true if file opened successfully
    bool start(const std::string &output_dir);

    /// Stop recording and finalize the file.
    void stop();

    /// Check if recording is active
    bool isRecording() const { return recording_.load(std::memory_order_relaxed); }

    /// Get the path of the current recording file
    std::string getFilePath() const { return file_path_; }

    /// Get number of events recorded so far
    uint64_t getEventCount() const { return event_count_.load(std::memory_order_relaxed); }

    // ═════════════════════════════════════════════════════════════════════════
    //                    RECORDING METHODS (called from producer threads)
    // ═════════════════════════════════════════════════════════════════════════

    /// Record a camera frame (will be JPEG-encoded on writer thread)
    void recordCamera(const CameraFrameData &frame, Mount mount);

    /// Record pre-encoded JPEG bytes (for ZMQ path — avoids re-encoding)
    void recordCameraJpeg(const uint8_t *jpeg, size_t len,
                          uint64_t timestamp_ns, Mount mount,
                          uint16_t width, uint16_t height);

    /// Record radar targets
    void recordRadar(const RadarTargets &targets);

    /// Record IMU sample
    void recordIMU(const ImuSample &sample);

    /// Record GPS data
    void recordGPS(float speed_mps, uint64_t ts_ms);

    /// Record canonical front-camera detections received from DeepStream IPC
    void recordFrontDetections(const DetBatch &batch);

  private:
    /// Writer thread loop — drains buffer and writes to disk
    void writerLoop();

    /// Enqueue an event for writing (thread-safe)
    void enqueue(RecordEvent &&event);

    // File state
    std::string file_path_;
    std::ofstream out_file_;

    // Writer thread
    std::thread writer_thread_;
    std::atomic<bool> recording_{false};
    std::atomic<bool> writer_stop_{false};

    // Thread-safe event buffer (mutex + vector, simple and reliable)
    std::mutex buffer_mutex_;
    std::vector<RecordEvent> write_buffer_;
    std::vector<RecordEvent> drain_buffer_; // Writer swaps to drain without holding lock

    // Stats
    std::atomic<uint64_t> event_count_{0};
    std::atomic<uint64_t> bytes_written_{0};
    uint64_t start_time_ns_ = 0;
};

} // namespace adas
