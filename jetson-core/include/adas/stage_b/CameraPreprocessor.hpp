// File: include/adas/stage_b/CameraPreprocessor.hpp
// Camera undistortion and preprocessing for Stage B
#pragma once

#include "adas/common/Types.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <optional>
#include <string>

namespace adas {

/// CameraIntrinsics loaded from calibration YAML
struct CameraIntrinsics {
    cv::Mat camera_matrix;      // 3x3 intrinsic matrix (fx, fy, cx, cy)
    cv::Mat dist_coeffs;        // Distortion coefficients (k1, k2, p1, p2, k3)
    int image_width;
    int image_height;
    double rms_error;
    bool valid = false;
};

/// CameraPreprocessor: Applies undistortion to camera frames
/// Loads calibration once at construction, applies cv::remap efficiently
class CameraPreprocessor {
  public:
    /// Constructor - loads calibration for the given mount
    /// @param mount Camera mount to load calibration for
    /// @param calib_dir Directory containing calibration YAML files
    explicit CameraPreprocessor(Mount mount, const std::string& calib_dir = "config/calibration");

    /// Check if calibration was loaded successfully
    bool isCalibrated() const { return calibrated_; }

    /// Get loaded intrinsics (for downstream use in projection)
    const CameraIntrinsics& getIntrinsics() const { return intrinsics_; }

    /// Get the valid image region after undistortion
    cv::Rect getROI() const { return roi_; }

    /// Process a frame: apply undistortion
    /// @param input Raw camera frame (BGR)
    /// @param crop_to_roi If true, crop result to valid ROI
    /// @return Undistorted frame (or input unchanged if not calibrated)
    cv::Mat process(const cv::Mat& input, bool crop_to_roi = false);

    /// Process frame data from queue
    /// @param input CameraFrameData from Stage A
    /// @param crop_to_roi If true, crop result to valid ROI
    /// @return Processed CameraFrameData with undistorted image
    CameraFrameData process(const CameraFrameData& input, bool crop_to_roi = false);

  private:
    bool loadCalibration(const std::string& yaml_path);
    void computeUndistortMaps();

    Mount mount_;
    CameraIntrinsics intrinsics_;
    bool calibrated_ = false;

    // Precomputed undistortion maps for cv::remap
    cv::Mat map_x_;
    cv::Mat map_y_;
    cv::Mat new_camera_matrix_;
    cv::Rect roi_;
};

} // namespace adas
