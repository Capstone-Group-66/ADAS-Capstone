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
        uint64_t rcw_packets = 0;
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
    /// @param rcw_queue Queue for compact RCW alert/status state
    /// @param imu_queue Queue for IMU samples
    /// @param radar_l_queue Queue for Rear L Radar targets
    /// @param radar_r_queue Queue for Rear R Radar targets
    /// @return true if connected successfully
    bool start(SPSCQueue<RcwState, 16> *rcw_queue, SPSCQueue<ImuSample, 32> *imu_queue,
               SPSCQueue<RadarTargets, 8> *radar_l_queue, SPSCQueue<RadarTargets, 8> *radar_r_queue);

    /// Stop receiving and disconnect
    void stop();

    /// Check if receiver is running
    bool isRunning() const { return running_.load(); }

    /// Check if Pi is connected (based on heartbeat)
    bool isPiConnected() const { return stats_.pi_connected; }

    /// Get statistics
    Stats getStats() const { return stats_; }

    /// Set recorder for data capture (optional, nullptr = no recording)
    /// Thread-safe: may be called from main thread while receive threads run.
    void setRecorder(Recorder *rec) {
      recorder_.store(rec, std::memory_order_release);
    }

    /// Get the most recent smoothed pitch angle received from Pi on port 5558.
    /// Written by imuThread when an ImuPitchRollPayload (8 bytes) is received.
    /// Returns 0.0f if no pitch message has arrived yet.
    float getLatestPitch() const {
        return latest_pitch_rad_.load(std::memory_order_relaxed);
    }

    /// Get the most recent smoothed roll angle received from Pi on port 5558.
    /// Written by imuThread when an ImuPitchRollPayload (8 bytes) is received.
    /// Returns 0.0f if no roll message has arrived yet.
    float getLatestRoll() const {
        return latest_roll_rad_.load(std::memory_order_relaxed);
    }

    /// Timestamp of the most recent pitch+roll packet used for Stage E attitude.
    /// This is the authoritative "IMU attitude is live" heartbeat for the
    /// camera-mounted Pi IMU path.
    uint64_t getLatestPitchRollTimeNs() const {
        return latest_pitch_roll_time_ns_.load(std::memory_order_relaxed);
    }

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
    void rcwThread();
    void radarLThread();
    void radarRThread();
    void imuThread();
    void heartbeatThread();

    /// Build ZMQ address
    std::string buildAddr(int port) const;

    // Configuration
    std::string pi_ip_;

    // ZMQ context and sockets (void* to avoid zmq.h in header)
    void *context_ = nullptr;
    void *rcw_socket_ = nullptr;
    void *radar_l_socket_ = nullptr;
    void *radar_r_socket_ = nullptr;
    void *imu_socket_ = nullptr;
    void *heartbeat_socket_ = nullptr;

    // Output queues (not owned)
    SPSCQueue<RcwState, 16> *rcw_queue_ = nullptr;
    SPSCQueue<ImuSample, 32> *imu_queue_ = nullptr;
    SPSCQueue<RadarTargets, 8> *radar_l_queue_ = nullptr;
    SPSCQueue<RadarTargets, 8> *radar_r_queue_ = nullptr;

    // Threads
    std::thread rcw_thread_;
    std::thread radar_l_thread_;
    std::thread radar_r_thread_;
    std::thread imu_thread_;
    std::thread heartbeat_thread_;

    // Control
    std::atomic<bool> running_{false};

    // Stats
    Stats stats_;

    // Sequence tracking for drop detection
    uint32_t last_rcw_seq_ = 0;
    uint32_t last_radar_l_seq_ = 0;
    uint32_t last_radar_r_seq_ = 0;
    uint32_t last_imu_seq_ = 0;

    // One-way network latency (RTT/2) in nanoseconds, measured at startup
    // and used to correct t_ingest_ns on ZMQ-received data
    std::atomic<uint64_t> one_way_latency_ns_{0};

    // Smoothed pitch + roll angles from Pi's ImuPitchRollPayload (8-byte message, port 5558).
    // Written by imuThread; read by the fusion layer via getLatestPitch()/getLatestRoll().
    std::atomic<float> latest_pitch_rad_{0.0f};
    std::atomic<float> latest_roll_rad_{0.0f};
    std::atomic<uint64_t> latest_pitch_roll_time_ns_{0};

    // Optional recorder for data capture
    // Atomic so it is safely visible across the camera/radar/IMU threads
    // without a mutex. Use memory_order_acquire to read.
    std::atomic<Recorder *> recorder_{nullptr};
};

} // namespace adas
