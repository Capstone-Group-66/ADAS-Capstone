// File: include/adas/stage_e/EgoFrame.hpp
// Kalman Filter for ego vehicle state estimation from IMU data
#pragma once

#include <cstdint>
#include <mutex>
#include <opencv2/video/tracking.hpp>

#include "adas/common/Types.hpp"

namespace adas {

/// EgoFrame: Kalman Filter for estimating ego vehicle state from IMU
///
/// State Vector (5D): [x, y, vx, vy, yaw]
/// - x, y: Accumulated position (relative, zeroed at start)
/// - vx, vy: Velocity from integrated accelerometer
/// - yaw: Heading from quaternion
///
/// This is "dead reckoning" - accuracy degrades over time without GPS.
/// For FCW, we primarily need instantaneous forward velocity (vx).
///
/// Usage:
///   EgoFrame ego;
///   ego.init();  // or ego = EgoFrame(initialState);
///   ego.update(imuSample, dt);
///   float speed = ego.getForwardVelocity_mps();
///
class EgoFrame {
  public:
    /// Default constructor - creates uninitialized EgoFrame
    EgoFrame();

    /// Constructor with initial state
    /// @param initialState 4x1 or 5x1 matrix [x, y, vx, vy, (yaw)]
    explicit EgoFrame(cv::Mat initialState);

    /// Initialize the Kalman filter with a state
    /// @param initialState Initial state vector
    void init(cv::Mat initialState);

    /// Initialize with zero state (stationary)
    void init();

    /// Predict the next state (Kalman predict step)
    /// @return Predicted state vector (5x1)
    cv::Mat getPrediction();

    /// Update with a new IMU sample
    /// @param sample IMU sample with accelerometer and quaternion data
    /// @param dt Time since last update (seconds)
    /// @return Corrected state estimate
    cv::Mat update(const ImuSample &sample, float dt);

    /// Update with raw measurement matrix
    /// @param measurement 4x1 measurement [x, y, vx, vy]
    /// @param dt Time since last update (seconds)
    /// @return Corrected state estimate
    cv::Mat update(cv::Mat measurement, float dt);

    /// Reset to zero state
    void reset();

    /// Get forward velocity (vx in ego frame)
    /// @return Speed in m/s (positive = forward)
    float getForwardVelocity_mps() const;

    /// Get lateral velocity (vy in ego frame)
    /// @return Speed in m/s (positive = left)
    float getLateralVelocity_mps() const;

    /// Get total speed magnitude
    /// @return Speed in m/s
    float getSpeed_mps() const;

    /// Get yaw (heading) from quaternion
    /// @return Yaw in radians
    float getYaw_rad() const;

    /// Correct velocity magnitude using GPS Doppler speed.
    /// Called at 1–5 Hz from BLE reader thread when GPS arrives from phone.
    /// Uses complementary filter: preserves IMU direction, corrects magnitude.
    /// @param gps_speed_mps GPS ground speed (no drift, ~0.1 m/s accuracy)
    void correctWithGpsSpeed(float gps_speed_mps);

    /// Check if GPS corrections are being received (not stale)
    bool hasRecentGps() const;

    /// Check if EgoFrame is initialized
    bool isInitialized() const { return kf_initialized; }

    /// Last update timestamp (nanoseconds)
    uint64_t previous_time_ns = 0;

  private:
    /// Update transition matrix with new dt
    void updateTransitionMatrix(float dt);

    /// Extract yaw from quaternion
    static float quaternionToYaw(float w, float x, float y, float z);

    cv::KalmanFilter kf;
    bool kf_initialized = false;

    // Cached values for getters
    float cached_vx_ = 0.0f;
    float cached_vy_ = 0.0f;
    float cached_yaw_ = 0.0f;

    // State and measurement dimensions
    static constexpr int stateDim = 5; // [x, y, vx, vy, yaw]
    static constexpr int measDim = 4;  // [x, y, vx, vy]

    // Thread safety: GPS corrections arrive from BLE reader thread,
    // while IMU updates come from the visualizer thread
    mutable std::mutex kf_mutex_;

    // GPS correction state
    float last_gps_speed_mps_ = 0.0f;
    uint64_t last_gps_time_ns_ = 0;
    static constexpr float GPS_CORRECTION_GAIN = 0.5f;
    static constexpr uint64_t GPS_STALE_NS = 3'000'000'000ULL;  // 3 seconds
};

} // namespace adas
