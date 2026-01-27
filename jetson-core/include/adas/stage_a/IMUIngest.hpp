// File: include/adas/stage_a/IMUIngest.hpp
// IMU ingest SCAFFOLD - awaiting BNO085 hardware arrival
// Implementation deferred per user directive
#pragma once

#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <atomic>
#include <string>
#include <thread>

namespace adas {

/// IMUIngest: SCAFFOLD ONLY - BNO085 hardware not yet arrived
///
/// This class structure is complete but implementation is deferred.
/// The BNO085 sensor will communicate via I2C or UART at ≥100 Hz.
///
/// When hardware arrives:
/// 1. Implement initBNO085() with proper I2C/UART initialization
/// 2. Implement readSample() to read real sensor data
/// 3. Test rate verification (FR4: ≥100 Hz)
///
class IMUIngest {
  public:
    /// Constructor
    /// @param queue Output SPSC queue for IMU samples
    /// @param config IMU configuration (bus, rate, protocol)
    IMUIngest(SPSCQueue<ImuSample, 32> &queue, const IMUConfig &config);

    /// Destructor
    ~IMUIngest();

    // Non-copyable
    IMUIngest(const IMUIngest &) = delete;
    IMUIngest &operator=(const IMUIngest &) = delete;

    /// Start ingest thread
    void start();

    /// Stop ingest thread
    void stop();

    /// Check if thread is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Check if IMU is healthy
    bool isHealthy() const { return healthy_.load(std::memory_order_relaxed); }

    /// Get statistics
    struct Stats {
        uint64_t samples_received;
        double rate_hz;
    };
    Stats getStats() const;

  private:
    /// Thread entry point
    void run();

    /// Initialize BNO085 sensor
    /// @return true if initialization successful
    bool initBNO085();

    /// Read single sample from sensor
    /// @return IMU sample with accelerometer and gyroscope data
    ImuSample readSample();

    SPSCQueue<ImuSample, 32> &queue_;
    IMUConfig config_;

    int fd_{-1};
    std::thread thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> healthy_{false};
    std::atomic<uint32_t> seq_{0};

    std::atomic<uint64_t> samples_received_{0};
    std::atomic<double> rate_hz_{0.0};
};

} // namespace adas
