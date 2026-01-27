// File: include/adas/common/Clock.hpp
// Unified clock wrapper for ADAS pipeline
// Per FR6: All sensors aligned to CLOCK_MONOTONIC_RAW timebase with ≤5ms skew
#pragma once

#include <cstdint>
#include <ctime>

namespace adas {

/// Unified clock wrapper using CLOCK_MONOTONIC_RAW
/// This is the AUTHORITATIVE time source for all sensor timestamps
class Clock {
  public:
    /// Get current time in nanoseconds
    /// This is the authoritative timestamp for all ingest operations
    static uint64_t now_ns() {
#ifdef _WIN32
        // Windows fallback for development (not used on Jetson)
        struct timespec ts;
        timespec_get(&ts, TIME_UTC);
        return static_cast<uint64_t>(ts.tv_sec) * NS_PER_SEC + static_cast<uint64_t>(ts.tv_nsec);
#else
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * NS_PER_SEC + static_cast<uint64_t>(ts.tv_nsec);
#endif
    }

    /// Get current time in milliseconds
    static uint64_t now_ms() { return now_ns() / NS_PER_MS; }

    /// Convert nanoseconds to milliseconds
    static constexpr uint64_t ns_to_ms(uint64_t ns) { return ns / NS_PER_MS; }

    /// Convert milliseconds to nanoseconds
    static constexpr uint64_t ms_to_ns(uint64_t ms) { return ms * NS_PER_MS; }

    /// Convert seconds to nanoseconds
    static constexpr uint64_t sec_to_ns(double sec) {
        return static_cast<uint64_t>(sec * static_cast<double>(NS_PER_SEC));
    }

    /// Convert nanoseconds to seconds
    static constexpr double ns_to_sec(uint64_t ns) {
        return static_cast<double>(ns) / static_cast<double>(NS_PER_SEC);
    }

    /// Compute elapsed time in nanoseconds
    static uint64_t elapsed_ns(uint64_t start_ns) {
        uint64_t now = now_ns();
        return (now >= start_ns) ? (now - start_ns) : 0;
    }

    /// Compute elapsed time in milliseconds
    static uint64_t elapsed_ms(uint64_t start_ns) { return ns_to_ms(elapsed_ns(start_ns)); }

    /// Check if a duration has elapsed since start
    static bool has_elapsed_ms(uint64_t start_ns, uint64_t duration_ms) {
        return elapsed_ns(start_ns) >= ms_to_ns(duration_ms);
    }

    // Time constants
    static constexpr uint64_t NS_PER_MS = 1'000'000ULL;
    static constexpr uint64_t NS_PER_SEC = 1'000'000'000ULL;
    static constexpr uint64_t MS_PER_SEC = 1'000ULL;

    // Pipeline timing constants (from componentConfig.yaml)
    static constexpr uint64_t FUSION_PERIOD_MS = 50; // 20 Hz
    static constexpr uint64_t MAX_SKEW_MS = 5;       // FR6
    static constexpr uint64_t BUFFER_MS = 150;       // Buffer depth
    static constexpr uint64_t LATE_DROP_MS = 100;    // Drop threshold
    static constexpr int WARMUP_TICKS = 40;          // ~2 seconds
};

} // namespace adas
