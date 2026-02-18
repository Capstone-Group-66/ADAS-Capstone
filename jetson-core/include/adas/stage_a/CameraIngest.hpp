// File: include/adas/stage_a/CameraIngest.hpp
// Direct USB camera capture for Stage A
// Handles Front, Side-L, and Side-R cameras connected via USB
#pragma once

#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <atomic>
#include <string>
#include <thread>

// Forward declare OpenCV types
namespace cv {
class VideoCapture;
class Mat;
} // namespace cv

namespace adas {

class Recorder; // Forward declaration for recording support

/// Camera ingest thread for direct USB cameras
/// Captures frames, timestamps them, and pushes to SPSC queue
class CameraIngest {
  public:
    /// Constructor
    /// @param mount Mount identity (FrontCam, SideCamL, SideCamR)
    /// @param device_path Device path from hardware_map.json (e.g., "/dev/video2")
    /// @param queue Output SPSC queue for frames
    /// @param config Camera configuration (resolution, FPS, MJPEG)
    CameraIngest(Mount mount, const std::string &device_path, SPSCQueue<CameraFrameData, 8> &queue,
                 const CameraConfig &config);

    /// Destructor - ensures thread is stopped
    ~CameraIngest();

    // Non-copyable
    CameraIngest(const CameraIngest &) = delete;
    CameraIngest &operator=(const CameraIngest &) = delete;

    /// Start capture thread
    void start();

    /// Stop capture thread gracefully
    void stop();

    /// Check if capture thread is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Check if camera is healthy (opened and capturing)
    bool isHealthy() const { return healthy_.load(std::memory_order_relaxed); }

    /// Get mount identity
    Mount getMount() const { return mount_; }

    /// Set recorder for data capture (optional, nullptr = no recording)
    void setRecorder(Recorder *rec) { recorder_ = rec; }

    /// Get device path
    const std::string &getDevicePath() const { return device_path_; }

    /// Get current sequence number
    uint32_t getSequence() const { return seq_.load(std::memory_order_relaxed); }

    /// Get frame rate statistics (last 100 frames)
    struct Stats {
        double fps_avg;
        double fps_min;
        double fps_max;
        uint64_t frames_captured;
        uint64_t drops;
    };
    Stats getStats() const;

  private:
    /// Thread entry point
    void run();

    /// Configure camera device (MJPEG, resolution, FPS)
    bool configureCamera();

    /// Capture single frame and push to queue
    bool captureFrame();

    Mount mount_;
    std::string device_path_;
    SPSCQueue<CameraFrameData, 8> &queue_;
    CameraConfig config_;

    std::unique_ptr<cv::VideoCapture> cap_;
    std::thread thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> healthy_{false};
    std::atomic<uint32_t> seq_{0};
    std::atomic<uint64_t> frames_captured_{0};

    // FPS tracking
    static constexpr size_t FPS_WINDOW = 100;
    std::array<uint64_t, FPS_WINDOW> frame_times_{};
    size_t frame_time_idx_ = 0;

    // Optional recorder for data capture
    Recorder *recorder_ = nullptr;
};

} // namespace adas
