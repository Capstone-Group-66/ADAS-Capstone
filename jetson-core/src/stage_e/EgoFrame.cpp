// File: src/stage_e/EgoFrame.cpp
// Kalman Filter implementation for ego vehicle state from IMU
#include "adas/stage_e/EgoFrame.hpp"

#include <cmath>
#include <iostream>

namespace adas {

EgoFrame::EgoFrame() {}

EgoFrame::EgoFrame(cv::Mat initialState) { init(initialState); }

void EgoFrame::init() {
  cv::Mat zeroState = (cv::Mat_<float>(stateDim, 1) << 0, 0, 0, 0, 0);
  init(zeroState);
}

void EgoFrame::init(cv::Mat initialState) {
  kf.init(stateDim, measDim, 0, CV_32F);

  float dt = 0.01f;  // Default 100Hz IMU rate

  // Transition matrix: constant velocity model
  kf.transitionMatrix =
      (cv::Mat_<float>(stateDim, stateDim) << 1, 0, dt, 0, 0, 0, 1, 0, dt, 0, 0,
       0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1);

  // Measurement matrix: we observe velocity and position
  kf.measurementMatrix = (cv::Mat_<float>(measDim, stateDim) << 1, 0, 0, 0, 0,
                          0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0);

  // Process noise - IMU integration is noisy
  cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-1));

  // Measurement noise - accelerometer has ~0.1 m/s^2 noise
  kf.measurementNoiseCov = (cv::Mat_<float>(measDim, measDim) << 1.0f, 0, 0,
                            0,               // x position (high uncertainty)
                            0, 1.0f, 0, 0,   // y position
                            0, 0, 0.01f, 0,  // vx (low noise after integration)
                            0, 0, 0, 0.01f);  // vy

  // Initial error covariance
  cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1.0));

  // Handle 4x1 input (add yaw = 0)
  if (initialState.rows == 4 && initialState.cols == 1) {
    cv::Mat fullState =
        (cv::Mat_<float>(stateDim, 1) << initialState.at<float>(0, 0),
         initialState.at<float>(1, 0), initialState.at<float>(2, 0),
         initialState.at<float>(3, 0), 0.0f);
    kf.statePost = fullState;
  } else if (initialState.rows == stateDim && initialState.cols == 1) {
    kf.statePost = initialState.clone();
  } else {
    std::cerr << "[EgoFrame] Invalid initial state dimensions\n";
    return;
  }

  cached_vx_ = kf.statePost.at<float>(2, 0);
  cached_vy_ = kf.statePost.at<float>(3, 0);
  cached_yaw_ = kf.statePost.at<float>(4, 0);

  kf_initialized = true;
}

cv::Mat EgoFrame::getPrediction() {
  if (!kf_initialized) {
    return cv::Mat();
  }
  return kf.predict();
}

cv::Mat EgoFrame::update(const ImuSample &sample, float dt) {
  if (!kf_initialized) {
    init();
  }

  // Guard against invalid dt
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f;  // Default to 10ms
  }

  // Extract acceleration (in sensor frame)
  float ax = sample.accel[0];  // Forward acceleration
  float ay = sample.accel[1];  // Lateral acceleration
  float az = sample.accel[2];  // Vertical (includes gravity ~9.81)

  // Remove gravity component from Z (assuming upright orientation)
  // For proper handling, use quaternion to rotate acceleration to world frame
  // Simplified: assume Z is up, subtract gravity
  (void)az;  // Not used for 2D velocity estimation

  // Get yaw from quaternion
  float yaw = quaternionToYaw(sample.quat[0], sample.quat[1], sample.quat[2],
                              sample.quat[3]);

  // Get current velocity estimate
  cv::Mat predicted = kf.predict();
  float vx_prev = predicted.at<float>(2, 0);
  float vy_prev = predicted.at<float>(3, 0);

  // Integrate acceleration to get velocity change
  // v_new = v_old + a * dt
  float vx_new = vx_prev + ax * dt;
  float vy_new = vy_prev + ay * dt;

  // Velocity deadband: clamp very small velocities to zero
  // This prevents drift when stationary
  if (std::abs(vx_new) < 0.1f) vx_new = 0.0f;
  if (std::abs(vy_new) < 0.1f) vy_new = 0.0f;

  // Integrate velocity to get position change
  float x_prev = predicted.at<float>(0, 0);
  float y_prev = predicted.at<float>(1, 0);
  float x_new = x_prev + vx_new * dt;
  float y_new = y_prev + vy_new * dt;

  // Build measurement vector
  cv::Mat measurement =
      (cv::Mat_<float>(measDim, 1) << x_new, y_new, vx_new, vy_new);

  // Update transition matrix with actual dt
  updateTransitionMatrix(dt);

  // Kalman correct
  cv::Mat estimated = kf.correct(measurement);

  // Cache values for getters
  cached_vx_ = estimated.at<float>(2, 0);
  cached_vy_ = estimated.at<float>(3, 0);
  cached_yaw_ = yaw;

  // Store yaw in state (not filtered, direct from quaternion)
  kf.statePost.at<float>(4, 0) = yaw;

  return estimated;
}

cv::Mat EgoFrame::update(cv::Mat measurement, float dt) {
  if (!kf_initialized) {
    init(measurement);
    return getPrediction();
  }

  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f;
  }

  updateTransitionMatrix(dt);
  kf.predict();
  cv::Mat estimated = kf.correct(measurement);

  cached_vx_ = estimated.at<float>(2, 0);
  cached_vy_ = estimated.at<float>(3, 0);

  return estimated;
}

void EgoFrame::reset() {
  kf_initialized = false;
  previous_time_ns = 0;
  cached_vx_ = 0.0f;
  cached_vy_ = 0.0f;
  cached_yaw_ = 0.0f;
}

float EgoFrame::getForwardVelocity_mps() const { return cached_vx_; }

float EgoFrame::getLateralVelocity_mps() const { return cached_vy_; }

float EgoFrame::getSpeed_mps() const {
  return std::sqrt(cached_vx_ * cached_vx_ + cached_vy_ * cached_vy_);
}

float EgoFrame::getYaw_rad() const { return cached_yaw_; }

void EgoFrame::updateTransitionMatrix(float dt) {
  kf.transitionMatrix.at<float>(0, 2) = dt;
  kf.transitionMatrix.at<float>(1, 3) = dt;
}

float EgoFrame::quaternionToYaw(float w, float x, float y, float z) {
  // Convert quaternion to yaw (rotation around Z axis)
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace adas
