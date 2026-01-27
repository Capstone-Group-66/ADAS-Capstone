// File: src/stage_e/Track.cpp
// Kalman Filter implementation for object tracking
#include "adas/stage_e/Track.hpp"

#include <iostream>

namespace adas {

Track::Track() {}

Track::Track(cv::Mat initialState) { init(initialState); }

void Track::init(cv::Mat initialState) {
    kf.init(stateDim, measDim, 0, CV_32F);

    float dt = 0.05f;  // Default elapsed time (50ms = 20Hz)

    // Transition matrix: predicts next state based on velocity
    // x_new = x + vx * dt
    // y_new = y + vy * dt
    // vx, vy, yaw assumed constant between updates
    kf.transitionMatrix =
        (cv::Mat_<float>(stateDim, stateDim) << 1, 0, dt, 0, 0, 0, 1, 0, dt, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1);

    // Measurement matrix: maps state to measurement
    // We measure [x, y, vx, vy] directly
    kf.measurementMatrix = (cv::Mat_<float>(measDim, stateDim) << 1, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0);

    // Process noise covariance (Q)
    // Higher values = trust measurements more, lower = trust model more
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));

    // Measurement noise covariance (R)
    // Based on sensor accuracy: position ~0.5m, velocity ~0.2m/s
    kf.measurementNoiseCov = (cv::Mat_<float>(measDim, measDim) << 0.25f, 0, 0,
                              0,                // x variance (0.5m std)
                              0, 0.25f, 0, 0,   // y variance
                              0, 0, 0.04f, 0,   // vx variance (0.2m/s std)
                              0, 0, 0, 0.04f);  // vy variance

    // Error covariance (P) - initial uncertainty
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1.0));

    // Handle 4x1 input (add yaw = 0)
    if (initialState.rows == 4 && initialState.cols == 1) {
        cv::Mat fullState =
            (cv::Mat_<float>(stateDim, 1) << initialState.at<float>(0, 0),
             initialState.at<float>(1, 0), initialState.at<float>(2, 0),
             initialState.at<float>(3, 0),
             0.0f);  // yaw = 0
        kf.statePost = fullState;
    } else if (initialState.rows == stateDim && initialState.cols == 1) {
        kf.statePost = initialState.clone();
    } else {
        std::cerr << "[Track] Invalid initial state dimensions: "
                  << initialState.rows << "x" << initialState.cols << "\n";
        return;
    }

    kf_initialized = true;
    object_detected = true;
    age_ = 0;
}

cv::Mat Track::getPrediction() {
    if (!kf_initialized) {
        return cv::Mat();
    }
    return kf.predict();
}

cv::Mat Track::update(cv::Mat measurement, float dt) {
    // Guard against invalid dt
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.05f;  // Default to 50ms
    }

    // Update transition matrix with actual dt
    updateTransitionMatrix(dt);

    if (!kf_initialized) {
        init(measurement);
        return getPrediction();
    }

    // Kalman predict + correct
    kf.predict();
    cv::Mat estimated = kf.correct(measurement);

    object_detected = true;
    age_++;

    return estimated;
}

void Track::reset() {
    kf_initialized = false;
    object_detected = false;
    previous_time_ns = 0;
    age_ = 0;
}

void Track::updateTransitionMatrix(float dt) {
    // Update position prediction based on velocity
    kf.transitionMatrix.at<float>(0, 2) = dt;  // x += vx * dt
    kf.transitionMatrix.at<float>(1, 3) = dt;  // y += vy * dt
}

}  // namespace adas
