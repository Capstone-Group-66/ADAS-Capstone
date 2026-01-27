// File: include/adas/stage_a/NetworkIngest.hpp
// TCP receiver for Raspberry Pi 4 rear sector data
// Handles RearCam, RearCornerRadarL, and RearCornerRadarR
#pragma once

#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

namespace adas {

/// NetworkIngest: TCP server receiving serialized sensor data from Pi4
///
/// The Raspberry Pi 4 acts as a serializer/forwarder for the rear sector:
/// - RearCam (MJPEG encoded frames)
/// - RearCornerRadarL (presence/range data)
/// - RearCornerRadarR (presence/range data)
///
/// Protocol: [NetPacketHeader (24 bytes) | Payload (variable)]
///
/// Timestamp Correction:
/// Since we don't use PTP, we apply a configurable latency correction:
///   t_ingest = t_arrival - latency_offset_ns
///
class NetworkIngest {
  public:
    /// Constructor
    /// @param cam_queue Output queue for RearCam frames
    /// @param radar_l_queue Output queue for RearCornerRadarL
    /// @param radar_r_queue Output queue for RearCornerRadarR
    /// @param config Network configuration (port, latency correction)
    NetworkIngest(SPSCQueue<CameraFrameData, 8> &cam_queue,
                  SPSCQueue<RadarTargets, 8> &radar_l_queue,
                  SPSCQueue<RadarTargets, 8> &radar_r_queue, const NetworkConfig &config);

    /// Destructor - ensures thread is stopped and sockets closed
    ~NetworkIngest();

    // Non-copyable
    NetworkIngest(const NetworkIngest &) = delete;
    NetworkIngest &operator=(const NetworkIngest &) = delete;

    /// Start network listener thread
    void start();

    /// Stop thread and close connections
    void stop();

    /// Check if listener is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Check if a client is connected
    bool isConnected() const { return connected_.load(std::memory_order_relaxed); }

    /// Check if receiving data successfully
    bool isHealthy() const { return healthy_.load(std::memory_order_relaxed); }

    /// Get statistics
    struct Stats {
        uint64_t packets_received;
        uint64_t bytes_received;
        uint64_t cam_frames;
        uint64_t radar_l_frames;
        uint64_t radar_r_frames;
        uint64_t errors;
        uint64_t reconnects;
    };
    Stats getStats() const;

  private:
    /// Thread entry point
    void run();

    /// Create listening socket
    bool createServerSocket();

    /// Accept incoming connection (blocking with timeout)
    bool acceptConnection();

    /// Read exact number of bytes from socket
    bool readExact(uint8_t *buffer, size_t length);

    /// Process received packet
    void handlePacket(const NetPacketHeader &header, const std::vector<uint8_t> &payload,
                      uint64_t t_arrival);

    /// Decode MJPEG payload to CameraFrameData
    CameraFrameData decodeCameraPacket(const uint8_t *payload, size_t size, uint64_t t_ingest);

    /// Parse radar payload to RadarTargets
    RadarTargets parseRadarPacket(const uint8_t *payload, size_t size, Mount mount,
                                  uint64_t t_ingest);

    /// Close client connection
    void closeClient();

    /// Close server socket
    void closeServer();

    // Queues
    SPSCQueue<CameraFrameData, 8> &cam_queue_;
    SPSCQueue<RadarTargets, 8> &radar_l_queue_;
    SPSCQueue<RadarTargets, 8> &radar_r_queue_;

    // Configuration
    NetworkConfig config_;
    uint64_t latency_offset_ns_;

    // Socket handles
    int server_fd_{-1};
    int client_fd_{-1};

    // Thread control
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> healthy_{false};

    // Sequence counters
    std::atomic<uint32_t> cam_seq_{0};
    std::atomic<uint32_t> radar_l_seq_{0};
    std::atomic<uint32_t> radar_r_seq_{0};

    // Statistics
    std::atomic<uint64_t> packets_received_{0};
    std::atomic<uint64_t> bytes_received_{0};
    std::atomic<uint64_t> cam_frames_{0};
    std::atomic<uint64_t> radar_l_frames_{0};
    std::atomic<uint64_t> radar_r_frames_{0};
    std::atomic<uint64_t> errors_{0};
    std::atomic<uint64_t> reconnects_{0};
};

} // namespace adas
