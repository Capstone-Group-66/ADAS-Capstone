// File: include/adas/stage_a/BSDReceiver.hpp
// ZMQ-based receiver for pure binary presence-mode Blind Spot Detection
#pragma once

#include "adas/common/Clock.hpp"
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace adas {

/// BSDReceiver: Dedicated module listening to binary ZMQ PULL streams
/// Parses explicit 32-byte binary structs and 1-byte payloads to extract 
/// the presence/absence of vehicles in the left/right blind spots.
class BSDReceiver {
  public:
    /// Constructor
    /// @param pi_ip IP address of the Pi publisher
    BSDReceiver(const std::string &pi_ip);

    /// Destructor - cleans up sockets and stops thread
    ~BSDReceiver();

    // Non-copyable
    BSDReceiver(const BSDReceiver &) = delete;
    BSDReceiver &operator=(const BSDReceiver &) = delete;

    /// Start the receiver thread
    bool start();

    /// Stop the receiver and disconnect sockets
    void stop();

    /// Check if receiver thread is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Get Left Side BSD presence state
    bool getLeftBSDState() const { return left_state_.load(std::memory_order_relaxed); }

    /// Get Right Side BSD presence state
    bool getRightBSDState() const { return right_state_.load(std::memory_order_relaxed); }

    /// Get dropped packet count on the left side
    uint32_t getLeftDrops() const { return left_drops_.load(std::memory_order_relaxed); }

    /// Get dropped packet count on the right side
    uint32_t getRightDrops() const { return right_drops_.load(std::memory_order_relaxed); }

    /// Packed schema for 32-byte header matching PiProtocol
#pragma pack(push, 1)
    struct BSDHeader {
        uint32_t magic;        // Must be 0x50493034
        uint16_t version;      // Must be 0x0100
        uint16_t msg_type;     // 0x0002 (Left) or 0x0003 (Right)
        uint32_t payload_size; // Must be 1
        uint32_t padding;      // Alignment padding
        uint64_t timestamp_ns;
        uint32_t sequence;
        uint32_t reserved;
    };
#pragma pack(pop)
    static_assert(sizeof(BSDHeader) == 32, "BSDHeader must be exactly 32 bytes");

  private:
    void receiveLoop();
    void processFailsafe(uint64_t now_ms);

    std::string pi_ip_;
    
    // ZMQ Context and sockets
    void *context_ = nullptr;
    void *left_socket_ = nullptr;
    void *right_socket_ = nullptr;

    // Threading
    std::thread thread_;
    std::atomic<bool> running_{false};

    // Shared State 
    std::atomic<bool> left_state_{false};
    std::atomic<bool> right_state_{false};

    // Telemetry and Tracking
    std::atomic<uint32_t> left_drops_{0};
    std::atomic<uint32_t> right_drops_{0};

    uint32_t left_last_seq_ = 0;
    uint32_t right_last_seq_ = 0;
    
    uint64_t left_last_rx_ms_ = 0;
    uint64_t right_last_rx_ms_ = 0;

    const uint64_t TIMEOUT_MS = 300;
};

} // namespace adas
