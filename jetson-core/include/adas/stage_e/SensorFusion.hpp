#include "adas/common/Clock.hpp"
#include "adas/common/Types.hpp"
#include "adas/stage_e/EgoFrame.hpp"
#include "adas/stage_e/Track.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <csignal>
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <thread>

namespace adas {

class SensorFusion {
public:
    /// Constructor
    /// @param frontRadarQueue Input SPSC queue for front radar targets
    /// @param imuQueue Input SPSC queue for IMU samples
    SensorFusion(SPSCQueue<RadarTargets, 8>& frontRadarQueue,
                    SPSCQueue<ImuSample, 32>& imuQueue);

    /// Destructor
    ~SensorFusion();

    // Non-copyable
    SensorFusion(const SensorFusion&) = delete;
    SensorFusion& operator=(const SensorFusion&) = delete;

    /// Start ingest thread
    void start();

    /// Stop ingest thread
    void stop();

    /// Check if thread is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Check if radar is healthy
    bool isHealthy() const { return healthy_.load(std::memory_order_relaxed); }

    /// Get statistics
    struct Stats {
        uint64_t frames_received;
        uint64_t bytes_received;
        uint64_t errors;
        double rate_hz;
    };
    Stats getStats() const;

private:
    /// Thread entry point
    void run();

    void updateFrontTrack();

    void updateEgoFrame();

    void detectFCW(cv::Mat ef, cv::Mat track);

    //Tracks and Kalman Filters
    Track frontTrack;
    EgoFrame egoFrame;

    //Input queues
    SPSCQueue<RadarTargets, 8>& FrontRadarQueue;
    SPSCQueue<ImuSample, 32>& IMUQueue;

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
};
}