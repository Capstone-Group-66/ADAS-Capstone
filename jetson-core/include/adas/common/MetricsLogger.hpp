// File: include/adas/common/MetricsLogger.hpp
// Lightweight metrics logger for validation and testing
#pragma once

#include <atomic>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace adas {

struct MetricEntry {
    uint64_t timestamp_ms;
    uint32_t frame_id;
    double inference_ms;
    float ttc_s;
    float range_m;
    bool fcw_triggered;
    double e2e_latency_ms;
};

class MetricsLogger {
public:
    MetricsLogger();
    ~MetricsLogger();

    // Enable/disable logging
    void enable();
    void disable();
    bool isEnabled() const;

    // Log a frame's metrics
    void logFrame(uint64_t timestamp_ms, uint32_t frame_id, 
                  double inference_ms, float ttc_s, float range_m,
                  bool fcw_triggered, double e2e_latency_ms);

    // Dump accumulated metrics to CSV
    bool dumpToCSV(const std::string& filepath);

    // Clear all logged data
    void clear();

    // Get current entry count
    size_t getEntryCount() const;

private:
    std::atomic<bool> enabled_{false};
    std::vector<MetricEntry> entries_;
    mutable std::mutex mutex_;
    
    static constexpr size_t MAX_ENTRIES = 10000;  // Prevent memory issues
};

}  // namespace adas
