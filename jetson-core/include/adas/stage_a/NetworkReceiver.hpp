// File: include/adas/stage_a/NetworkReceiver.hpp
// ZMQ-based receiver for Pi4 sensor data, integrated with pipeline queues
#pragma once

#include "adas/common/Config.hpp"
#include "adas/common/PiProtocol.hpp"
#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// Forward declare ZMQ context to avoid header pollution
typedef void *zmq_context_t;

namespace adas {

class Recorder; // Forward declaration for recording support

/// NetworkReceiver: Receives sensor data from Pi4 via ZMQ
/// Integrates directly with Stage A queues
class NetworkReceiver {
  public:
    /// Statistics
    struct Stats {
        uint64_t cam_frames = 0;
        uint64_t radar_l_packets = 0;
        uint64_t radar_r_packets = 0;
        uint64_t imu_samples = 0;
        uint64_t heartbeats = 0;
        uint64_t errors = 0;
        uint64_t drops = 0;
        int64_t last_chrony_offset_us = 0;
        bool pi_connected = false;
    };

    /// Discovery result for a single device
    struct DiscoveredDevice {
        protocol::DeviceType type;
        protocol::MountId mount;
        protocol::DeviceStatus status;
        std::string serial;
    };

    /// Constructor
    /// @param pi_ip IP address of Pi4
    NetworkReceiver(const std::string &pi_ip);

    /// Destructor - stops all threads and cleans up ZMQ
    ~NetworkReceiver();

    // Non-copyable
    NetworkReceiver(const NetworkReceiver &) = delete;
    NetworkReceiver &operator=(const NetworkReceiver &) = delete;

    /// Connect and start receiving data
    /// @param cam_queue Queue for RearCam frames
    /// @param imu_queue Queue for IMU samples
    /// @return true if connected successfully
    bool start(SPSCQueue<CameraFrameData, 8> *cam_queue, SPSCQueue<ImuSample, 32> *imu_queue);

    /// Stop receiving and disconnect
    void stop();

    /// Check if receiver is running
    bool isRunning() const { return running_.load(); }

    /// Check if Pi is connected (based on heartbeat)
    bool isPiConnected() const { return stats_.pi_connected; }

    /// Get statistics
    Stats getStats() const { return stats_; }

    /// Set recorder for data capture (optional, nullptr = no recording)
    void setRecorder(Recorder *rec) { recorder_ = rec; }

    /// Static: Discover devices on Pi without starting full receiver
    /// @param pi_ip IP address of Pi4
    /// @param timeout_ms Timeout in milliseconds
    /// @return List of discovered devices, empty if failed
    static std::vector<DiscoveredDevice> discoverDevices(const std::string &pi_ip,
                                                         int timeout_ms = 3000);

    /// Static: Test RTT using ZMQ (more accurate than ping)
    /// @param pi_ip IP address of Pi4
    /// @return RTT in milliseconds, or -1 if failed
    static double measureRTT(const std::string &pi_ip);

  private:
    /// Thread functions
    void cameraThread();
    void radarLThread();
    void radarRThread();
    void imuThread();
    void heartbeatThread();

    /// Process received message
    bool processMessage(const uint8_t *data, size_t len, protocol::MessageType expected);

    /// Build ZMQ address
    std::string buildAddr(int port) const;

    // Configuration
    std::string pi_ip_;

    // ZMQ context and sockets (void* to avoid zmq.h in header)
    void *context_ = nullptr;
    void *cam_socket_ = nullptr;
    void *radar_l_socket_ = nullptr;
    void *radar_r_socket_ = nullptr;
    void *imu_socket_ = nullptr;
    void *heartbeat_socket_ = nullptr;

    // Output queues (not owned)
    SPSCQueue<CameraFrameData, 8> *cam_queue_ = nullptr;
    SPSCQueue<ImuSample, 32> *imu_queue_ = nullptr;

    // Threads
    std::thread cam_thread_;
    std::thread radar_l_thread_;
    std::thread radar_r_thread_;
    std::thread imu_thread_;
    std::thread heartbeat_thread_;

    // Control
    std::atomic<bool> running_{false};

    // Stats
    Stats stats_;

    // Sequence tracking for drop detection
    uint32_t last_cam_seq_ = 0;
    uint32_t last_radar_l_seq_ = 0;
    uint32_t last_radar_r_seq_ = 0;
    uint32_t last_imu_seq_ = 0;

    // One-way network latency (RTT/2) in nanoseconds, measured at startup
    // and used to correct t_ingest_ns on ZMQ-received data
    std::atomic<uint64_t> one_way_latency_ns_{0};

    // Optional recorder for data capture
    Recorder *recorder_ = nullptr;
};

} // namespace adas
