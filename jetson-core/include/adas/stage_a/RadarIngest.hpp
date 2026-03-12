// File: include/adas/stage_a/RadarIngest.hpp
// Direct serial radar capture for OPS243-A front radar
// Based on test_scripts/radar_freq_test.cpp patterns
#pragma once

#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <string>
#include <thread>
#include <vector>
#include <fstream>

namespace adas {

class Recorder; // Forward declaration for recording support

/// RadarIngest: Serial port reader for OPS243-A front radar
///
/// The OPS243-A outputs target data over serial at 921600 baud.
/// Based on test_scripts/radar_freq_test.cpp which demonstrated ≥20Hz data rate.
///
class RadarIngest {
  public:
    /// Constructor
    /// @param mount Mount identity (should be FrontRadar)
    /// @param port Serial port path (e.g., "/dev/ttyACM0")
    /// @param queue Output SPSC queue for radar targets
    /// @param config Radar configuration (baud rate, timeout)
    RadarIngest(Mount mount, const std::string &port, SPSCQueue<RadarTargets, 8> &queue,
                const RadarConfig &config);

    /// Destructor
    ~RadarIngest();

    // Non-copyable
    RadarIngest(const RadarIngest &) = delete;
    RadarIngest &operator=(const RadarIngest &) = delete;

    /// Start ingest thread
    void start();

    /// Stop ingest thread
    void stop();

    /// Check if thread is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Check if radar is healthy
    bool isHealthy() const { return healthy_.load(std::memory_order_relaxed); }

    /// Get mount identity
    Mount getMount() const { return mount_; }

    /// Set recorder for data capture (optional, nullptr = no recording)
    void setRecorder(Recorder *rec) { recorder_ = rec; }

    /// Get statistics
    struct Stats {
        uint64_t frames_received;
        uint64_t bytes_received;
        uint64_t errors;
        double rate_hz;
        double range_hz;
        double speed_event_hz;
        double fused_with_fresh_speed_ratio;
        double stale_speed_ratio;
        uint64_t parse_error_count;
    };
    Stats getStats() const;

    /// Parse raw bytes into RadarTargets (public for unit testing)
    RadarTargets parseFrame(const uint8_t *data, size_t len, uint64_t t_ingest);

  private:
    /// Thread entry point
    void run();

    /// Setup serial port with termios
    bool setupSerialPort();

    /// Read available bytes from serial.
    /// Return value:
    ///   1  = data read
    ///   0  = timeout/no data
    ///  -1  = recoverable read error (keep fd open)
    ///  -2  = disconnect/re-enumeration detected (fd should be reopened)
    int readFrame(std::vector<uint8_t> &buffer);

    /// Close active serial fd if open.
    void closeSerialFd();

    Mount mount_;
    std::string port_;
    SPSCQueue<RadarTargets, 8> &queue_;
    RadarConfig config_;

    int fd_{-1};
    std::thread thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> healthy_{false};
    std::atomic<uint32_t> seq_{0};

    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> bytes_received_{0};
    std::atomic<uint64_t> errors_{0};

    // Rate calculation
    uint64_t last_rate_time_{0};
    uint64_t frames_in_window_{0};
    std::atomic<double> rate_hz_{0.0};

    // Extended Stats
    std::atomic<uint64_t> parse_error_count_{0};
    std::atomic<uint64_t> range_events_window_{0};
    std::atomic<uint64_t> speed_events_window_{0};
    std::atomic<uint64_t> fused_fresh_window_{0};
    std::atomic<uint64_t> fused_stale_window_{0};
    
    std::atomic<double> range_hz_{0.0};
    std::atomic<double> speed_event_hz_{0.0};
    std::atomic<double> fused_with_fresh_speed_ratio_{0.0};
    std::atomic<double> stale_speed_ratio_{0.0};

    // Fusion holding parameters
    float last_speed_mps_{0.0f};
    uint64_t last_speed_ts_monotonic_{0};
    std::string line_buffer_; // For assembling JSON lines
    bool combined_native_mode_{false};

    // Reconnect/watchdog
    uint64_t last_data_time_ns_{0};
    uint64_t last_connect_attempt_ns_{0};
    uint64_t connected_since_ns_{0};
    uint64_t frames_at_connect_{0};
    uint32_t reconnect_backoff_ms_{200};
    int desired_baud_rate_{921600};
    int active_baud_rate_{0};
    bool startup_baud_probe_done_{false};

    // Raw CSV Logger
    std::ofstream raw_csv_file_;

    // Optional recorder for data capture
    Recorder *recorder_ = nullptr;
};

} // namespace adas
