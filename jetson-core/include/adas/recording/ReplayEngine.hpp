// File: include/adas/recording/ReplayEngine.hpp
// Replay .adasrec files into SPSC queues for offline analysis
#pragma once

#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"
#include "adas/recording/Recorder.hpp" // For file format types

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace adas {

/// ReplayEngine: reads .adasrec file and pushes events into pipeline queues
/// with accurate wall-clock pacing.
class ReplayEngine {
  public:
    ReplayEngine() = default;
    ~ReplayEngine();

    // Non-copyable
    ReplayEngine(const ReplayEngine &) = delete;
    ReplayEngine &operator=(const ReplayEngine &) = delete;

    /// Load a .adasrec file into memory
    /// @return true if file is valid and loaded
    bool load(const std::string &path);

    /// Start replay (launches replay thread)
    /// Must connect the required queues/callbacks first
    void start();

    /// Stop replay
    void stop();

    /// Check if replay is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Check if replay finished naturally (hit end of file)
    bool isFinished() const { return finished_.load(std::memory_order_relaxed); }

    /// Set replay speed multiplier (1.0 = realtime, 0.0 = fast-as-possible)
    void setSpeed(float speed) { speed_ = speed; }

    /// Get current progress (0.0 to 1.0)
    float getProgress() const;

    /// Get total event count
    size_t getEventCount() const { return events_.size(); }

    /// Get total duration in nanoseconds
    uint64_t getDurationNs() const;

    // ═════════════════════════════════════════════════════════════════════════
    //                    QUEUE CONNECTIONS (set before start())
    // ═════════════════════════════════════════════════════════════════════════

    void setCameraQueue(Mount mount, SPSCQueue<CameraFrameData, 8> *queue);
    void setFrontDetQueue(SPSCQueue<DetBatch, 8> *queue);
    void setRadarQueue(Mount mount, SPSCQueue<RadarTargets, 8> *queue);
    void setIMUQueue(SPSCQueue<ImuSample, 32> *queue);
    void setGpsCallback(std::function<void(float, uint64_t)> cb);

  private:
    /// Replay thread main loop
    void replayLoop();

    /// Dispatch a single event to the appropriate queue
    void dispatchEvent(const RecordEvent &event);

    /// Decode camera event payload → CameraFrameData and push to queue
    void dispatchCamera(const RecordEvent &event);

    /// Decode front detection payload → DetBatch and push to queue
    void dispatchFrontDetBatch(const RecordEvent &event);

    /// Decode radar event payload → RadarTargets and push to queue
    void dispatchRadar(const RecordEvent &event);

    /// Decode IMU event payload → ImuSample and push to queue
    void dispatchIMU(const RecordEvent &event);

    /// Decode GPS event payload and call callback
    void dispatchGPS(const RecordEvent &event);

    // Loaded events (sorted by timestamp)
    std::vector<RecordEvent> events_;
    AdasRecFileHeader file_header_;
    uint64_t replay_start_time_ns_ = 0;
    uint64_t first_event_ts_ = 0;

    // Queue pointers
    SPSCQueue<CameraFrameData, 8> *cam_side_l_queue_ = nullptr;
    SPSCQueue<CameraFrameData, 8> *cam_side_r_queue_ = nullptr;
    SPSCQueue<CameraFrameData, 8> *cam_rear_queue_ = nullptr;
    SPSCQueue<DetBatch, 8> *front_det_queue_ = nullptr;
    SPSCQueue<RadarTargets, 8> *radar_front_queue_ = nullptr;
    SPSCQueue<RadarTargets, 8> *radar_rear_l_queue_ = nullptr;
    SPSCQueue<RadarTargets, 8> *radar_rear_r_queue_ = nullptr;
    SPSCQueue<ImuSample, 32> *imu_queue_ = nullptr;
    std::function<void(float, uint64_t)> gps_callback_;

    // Control
    std::thread replay_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> finished_{false};
    std::atomic<size_t> current_index_{0};
    float speed_ = 1.0f;
};

} // namespace adas
