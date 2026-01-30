// File: src/common/MetricsLogger.cpp
// Implementation of metrics logger for validation
#include "adas/common/MetricsLogger.hpp"
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace adas {

MetricsLogger::MetricsLogger() { entries_.reserve(MAX_ENTRIES); }

MetricsLogger::~MetricsLogger() = default;

void MetricsLogger::enable() {
  enabled_.store(true);
  std::cout << "[MetricsLogger] Logging enabled\n";
}

void MetricsLogger::disable() {
  enabled_.store(false);
  std::cout << "[MetricsLogger] Logging disabled\n";
}

bool MetricsLogger::isEnabled() const { return enabled_.load(); }

void MetricsLogger::logFrame(uint64_t timestamp_ms, uint32_t frame_id,
                             double inference_ms, float ttc_s, float range_m,
                             bool fcw_triggered, double e2e_latency_ms) {
  if (!enabled_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (entries_.size() >= MAX_ENTRIES) {
    // Drop oldest entries if at capacity (FIFO)
    entries_.erase(entries_.begin());
  }

  entries_.push_back({timestamp_ms, frame_id, inference_ms, ttc_s, range_m,
                      fcw_triggered, e2e_latency_ms});
}

bool MetricsLogger::dumpToCSV(const std::string &filepath) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (entries_.empty()) {
    std::cerr << "[MetricsLogger] No data to dump\n";
    return false;
  }

  std::ofstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "[MetricsLogger] Failed to open: " << filepath << "\n";
    return false;
  }

  // Write CSV header
  file << "timestamp_ms,frame_id,inference_ms,ttc_s,range_m,fcw_triggered,e2e_"
          "latency_ms\n";

  // Write data rows
  for (const auto &entry : entries_) {
    file << entry.timestamp_ms << "," << entry.frame_id << "," << std::fixed
         << std::setprecision(2) << entry.inference_ms << "," << std::fixed
         << std::setprecision(3) << entry.ttc_s << "," << std::fixed
         << std::setprecision(2) << entry.range_m << ","
         << (entry.fcw_triggered ? 1 : 0) << "," << std::fixed
         << std::setprecision(2) << entry.e2e_latency_ms << "\n";
  }

  file.close();
  std::cout << "[MetricsLogger] Wrote " << entries_.size()
            << " entries to: " << filepath << "\n";
  return true;
}

void MetricsLogger::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.clear();
  std::cout << "[MetricsLogger] Cleared all entries\n";
}

size_t MetricsLogger::getEntryCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

} // namespace adas
