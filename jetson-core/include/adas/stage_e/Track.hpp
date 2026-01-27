// File: include/adas/stage_e/Track.hpp
// Kalman Filter wrapper for tracked objects
// Adapted from John's work with fixes for noise covariances and thread safety
#pragma once

#include <cstdint>
#include <opencv2/video/tracking.hpp>

namespace adas {

/// Track: Kalman Filter for tracking a single detected object over time
///
/// State Vector (5D): [x, y, vx, vy, yaw]
/// - x, y: Position in ego frame (meters)
/// - vx, vy: Velocity (m/s)
/// - yaw: Heading (radians) - reserved for future use
///
/// Measurement Vector (4D): [x, y, vx, vy]
/// - Direct observations from fused camera+radar data
///
/// Usage:
///   Track track;  // Uninitialized
///   track = Track(initialState);  // Initialize with first measurement
///   track.update(measurement, dt);  // Update with new measurement
///   cv::Mat prediction = track.getPrediction();  // Get predicted state
///
class Track {
   public:
    /// Default constructor - creates uninitialized track
    Track();

    /// Constructor with initial state
    /// @param initialState 4x1 or 5x1 matrix [x, y, vx, vy, (yaw)]
    explicit Track(cv::Mat initialState);

    /// Initialize the Kalman filter with a state
    /// @param initialState Initial state vector
    void init(cv::Mat initialState);

    /// Predict the next state (Kalman predict step)
    /// @return Predicted state vector (5x1)
    cv::Mat getPrediction();

    /// Update with a new measurement (Kalman correct step)
    /// @param measurement 4x1 measurement [x, y, vx, vy]
    /// @param dt Time since last update (seconds)
    /// @return Corrected state estimate
    cv::Mat update(cv::Mat measurement, float dt);

    /// Reset the track to uninitialized state
    void reset();

    /// Check if track is initialized
    bool isInitialized() const { return kf_initialized; }

    /// Check if object was detected in recent frames
    bool hasObject() const { return object_detected; }

    /// Get track age (number of updates since init)
    uint32_t getAge() const { return age_; }

    /// Last update timestamp (for staleness checks)
    uint64_t previous_time_ns = 0;

    /// Flag indicating if object was detected (set by update, cleared by reset)
    bool object_detected = false;

   private:
    /// Update transition matrix with new dt
    void updateTransitionMatrix(float dt);

    cv::KalmanFilter kf;
    bool kf_initialized = false;
    uint32_t age_ = 0;

    // State and measurement dimensions
    static constexpr int stateDim = 5;  // [x, y, vx, vy, yaw]
    static constexpr int measDim = 4;   // [x, y, vx, vy]
};

}  // namespace adas
