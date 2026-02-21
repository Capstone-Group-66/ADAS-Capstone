// File: include/adas/stage_a/IngestManager.hpp
// Lifecycle manager for all Stage A ingest threads
#pragma once

#include "adas/common/Config.hpp"
#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"
#include "adas/stage_a/CameraIngest.hpp"
#include "adas/stage_a/NetworkIngest.hpp"
#include "adas/stage_a/RadarIngest.hpp"
#include "adas/recording/ReplayEngine.hpp"

#ifdef HAS_ZMQ
#include "adas/stage_a/NetworkReceiver.hpp"
#endif

#include <map>
#include <memory>
#include <thread>
#include <vector>

namespace adas {

class Recorder; // Forward declaration for recording support

/// IngestManager: Lifecycle controller for all Stage A ingest threads
///
/// Responsibilities:
/// - Own and manage all SPSC queues
/// - Create and start/stop all ingest threads
/// - Provide queue access for downstream stages (Stage B, C, E)
/// - Monitor aggregate health status
/// - Implement graceful shutdown (FR93)
///
class IngestManager {
  public:
    /// Constructor
    /// @param config Pipeline configuration
    /// @param hw_map Hardware device mapping (from DeviceWizard)
    IngestManager(const Config &config, const HardwareMap &hw_map);

    /// Destructor - stops all threads
    ~IngestManager();

    // Non-copyable
    IngestManager(const IngestManager &) = delete;
    IngestManager &operator=(const IngestManager &) = delete;
    
    /// Initialize in Replay Mode
    /// @param file_path Path to the .adasrec file
    /// @param speed Playback speed multiplier (1.0 = realtime)
    /// @return true if file loaded successfully
    bool initReplay(const std::string& file_path, float speed = 1.0f);

    /// Start all ingest threads (or replay engine)
    void start();

    /// Stop all threads gracefully (FR93)
    void stop();

    /// Check if all ingest threads are running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }
    
    /// Check if running in Replay Mode
    bool isReplayMode() const { return is_replay_mode_; }

    /// Set recorder for all ingest threads (call before or after start)
    void setRecorder(Recorder *recorder);

    // ═══════════════════════════════════════════════════════════════════════════
    //                        QUEUE ACCESS FOR DOWNSTREAM
    // ═══════════════════════════════════════════════════════════════════════════

    /// Get camera queue by mount
    /// @throws std::out_of_range if mount is not a camera
    SPSCQueue<CameraFrameData, 8> &getCameraQueue(Mount mount);

    /// Get radar queue by mount
    /// @throws std::out_of_range if mount is not a radar
    SPSCQueue<RadarTargets, 8> &getRadarQueue(Mount mount);

    /// Get IMU queue
    SPSCQueue<ImuSample, 32> &getIMUQueue() { return imu_queue_; }

    // ═══════════════════════════════════════════════════════════════════════════
    //                             HEALTH MONITORING
    // ═══════════════════════════════════════════════════════════════════════════

    /// Aggregate health status
    struct HealthStatus {
        bool all_healthy;
        std::map<Mount, bool> sensor_health;
        uint64_t total_drops;
        std::string summary;
    };

    /// Get current health status
    HealthStatus getHealth() const;

    /// Print status to stdout
    void printStatus() const;

  private:
    void launchDirectCameras();
    void launchNetworkIngest();
    void launchFrontRadar();

    Config config_;
    HardwareMap hw_map_;

    // ═══════════════════════════════════════════════════════════════════════════
    //                    QUEUES (Preallocated, owned by manager)
    // ═══════════════════════════════════════════════════════════════════════════

    // Camera queues (capacity 8 each per spec)
    SPSCQueue<CameraFrameData, 8> cam_front_queue_;
    SPSCQueue<CameraFrameData, 8> cam_side_l_queue_;
    SPSCQueue<CameraFrameData, 8> cam_side_r_queue_;
    SPSCQueue<CameraFrameData, 8> cam_rear_queue_;

    // Radar queues (capacity 8 each per spec)
    SPSCQueue<RadarTargets, 8> radar_front_queue_;
    SPSCQueue<RadarTargets, 8> radar_rear_l_queue_;
    SPSCQueue<RadarTargets, 8> radar_rear_r_queue_;

    // IMU queue (capacity 32 for high-rate data)
    SPSCQueue<ImuSample, 32> imu_queue_;

    // ═══════════════════════════════════════════════════════════════════════════
    //                           INGEST INSTANCES
    // ═══════════════════════════════════════════════════════════════════════════

    // Direct USB cameras
    std::unique_ptr<CameraIngest> cam_front_;
    std::unique_ptr<CameraIngest> cam_side_l_;
    std::unique_ptr<CameraIngest> cam_side_r_;

    // Network ingest (rear sector from Pi4)
    std::unique_ptr<NetworkIngest> network_;

#ifdef HAS_ZMQ
    // ZMQ-based network receiver (preferred over TCP NetworkIngest)
    std::unique_ptr<NetworkReceiver> zmq_receiver_;
#endif

    // Direct front radar
    std::unique_ptr<RadarIngest> radar_front_;
    
    // Replay Engine (replaces hardware ingestors when active)
    std::unique_ptr<ReplayEngine> replay_engine_;

    std::atomic<bool> running_{false};
    bool is_replay_mode_{false};
};

} // namespace adas
