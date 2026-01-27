// File: src/stage_a/IMUIngest.cpp
// IMU ingest SCAFFOLD - awaiting BNO085 hardware arrival
// Implementation deferred per user directive
#include "adas/stage_a/IMUIngest.hpp"

#include <cmath>
#include <iostream>

namespace adas {

IMUIngest::IMUIngest(SPSCQueue<ImuSample, 32> &queue, const IMUConfig &config)
    : queue_(queue), config_(config) {}

IMUIngest::~IMUIngest() { stop(); }

void IMUIngest::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }

    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&IMUIngest::run, this);
}

void IMUIngest::stop() {
    running_.store(false, std::memory_order_relaxed);

    if (thread_.joinable()) {
        thread_.join();
    }
}

void IMUIngest::run() {
    std::cout << "[IMUIngest] Starting (SCAFFOLD MODE - hardware pending)\n";
    std::cout << "[IMUIngest] Config: bus=" << config_.bus << ", rate=" << config_.rate_hz << " Hz"
              << ", uart=" << (config_.use_uart ? "true" : "false") << std::endl;

    // SCAFFOLD: Try to initialize, expect failure until hardware arrives
    if (!initBNO085()) {
        std::cerr << "[IMUIngest] BNO085 not found - hardware pending arrival\n";
        std::cerr << "[IMUIngest] Running in STUB mode (no real data)\n";
        healthy_.store(false, std::memory_order_relaxed);

        // In stub mode, just sleep and wait for shutdown
        while (running_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return;
    }

    // If somehow we do connect (unlikely until hardware arrives)
    healthy_.store(true, std::memory_order_relaxed);
    uint64_t last_rate_time = Clock::now_ns();
    uint64_t samples_in_window = 0;

    // Target period in nanoseconds (e.g., 100 Hz = 10ms = 10,000,000 ns)
    uint64_t target_period_ns = Clock::sec_to_ns(1.0 / config_.rate_hz);

    while (running_.load(std::memory_order_relaxed)) {
        uint64_t loop_start = Clock::now_ns();

        // Read sample
        ImuSample sample = readSample();

        // Push to queue
        queue_.try_push(std::move(sample));
        samples_received_.fetch_add(1, std::memory_order_relaxed);
        samples_in_window++;

        // Calculate rate every 5 seconds
        if (Clock::elapsed_ms(last_rate_time) >= 5000) {
            double elapsed_sec = Clock::ns_to_sec(Clock::now_ns() - last_rate_time);
            double hz = static_cast<double>(samples_in_window) / elapsed_sec;
            rate_hz_.store(hz, std::memory_order_relaxed);

            std::cout << "[IMUIngest] rate: " << hz << " Hz"
                      << (hz >= 100.0 ? " [PASS]" : " [WARN: below 100Hz]") << std::endl;

            samples_in_window = 0;
            last_rate_time = Clock::now_ns();
        }

        // Sleep to maintain target rate
        uint64_t elapsed = Clock::now_ns() - loop_start;
        if (elapsed < target_period_ns) {
            uint64_t sleep_ns = target_period_ns - elapsed;
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        }
    }

    std::cout << "[IMUIngest] Stopped\n";
}

bool IMUIngest::initBNO085() {
    // =========================================================================
    // SCAFFOLD: BNO085 INITIALIZATION - TO BE IMPLEMENTED WHEN HARDWARE ARRIVES
    // =========================================================================
    //
    // Implementation notes (for when hardware arrives):
    //
    // 1. I2C Mode (default):
    //    - Open /dev/i2c-X
    //    - BNO085 default address: 0x4A (or 0x4B)
    //    - Use Adafruit BNO08x library patterns or SH2 protocol directly
    //
    // 2. UART Mode:
    //    - Open /dev/ttyTHS1 or similar
    //    - Configure baud rate (typically 3000000 for BNO085)
    //    - Use SHTP UART protocol
    //
    // 3. Initialization sequence:
    //    - Reset device
    //    - Read product ID
    //    - Enable rotation vector / accelerometer / gyroscope reports
    //    - Set report interval (10ms for 100Hz)
    //
    // 4. Expected data:
    //    - Accelerometer: ax, ay, az in m/s^2
    //    - Gyroscope: wx, wy, wz in rad/s
    //    - (Optional) Rotation vector for orientation
    //
    // =========================================================================

    std::cerr << "[IMUIngest] initBNO085() - NOT IMPLEMENTED (hardware pending)\n";
    return false; // Always fail until implemented
}

ImuSample IMUIngest::readSample() {
    // =========================================================================
    // SCAFFOLD: BNO085 DATA READ - TO BE IMPLEMENTED WHEN HARDWARE ARRIVES
    // =========================================================================
    //
    // This should:
    // 1. Read accelerometer/gyroscope data from sensor
    // 2. Convert to proper units (m/s^2, rad/s)
    // 3. Apply timestamp
    //
    // =========================================================================

    uint64_t t_ingest = Clock::now_ns();
    seq_.fetch_add(1, std::memory_order_relaxed);

    ImuSample sample;
    sample.t_capture = t_ingest;
    sample.accel = {0.0f, 0.0f, 9.81f}; // Stub: gravity on Z
    sample.gyro = {0.0f, 0.0f, 0.0f};
    sample.mag = {0.0f, 0.0f, 0.0f};
    sample.quat = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion
    sample.temperature = 25.0f;
    sample.calibration_status = 0; // Not calibrated (stub)

    return sample;
}

IMUIngest::Stats IMUIngest::getStats() const {
    Stats s;
    s.samples_received = samples_received_.load(std::memory_order_relaxed);
    s.rate_hz = rate_hz_.load(std::memory_order_relaxed);
    return s;
}

} // namespace adas
