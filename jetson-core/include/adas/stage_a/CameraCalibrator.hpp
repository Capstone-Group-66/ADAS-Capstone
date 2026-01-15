// File: include/adas/stage_a/CameraCalibrator.hpp
// Camera intrinsic calibration using OpenCV chessboard pattern
#pragma once

#include "adas/common/Types.hpp"

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace adas {

/// Camera intrinsic calibration data
struct CameraIntrinsics {
    cv::Mat camera_matrix;           ///< 3x3 camera matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    cv::Mat distortion_coefficients; ///< 1x5 distortion [k1, k2, p1, p2, k3]
    int image_width;
    int image_height;
    double rms_error;          ///< Root mean square reprojection error
    std::string calibrated_at; ///< ISO 8601 timestamp
    cv::Size pattern_size;     ///< Inner corners (e.g., 9x6)
    double square_size_m;      ///< Square size in meters
};

/// CameraCalibrator: Interactive chessboard-based camera calibration
/// Uses OpenCV's cv::calibrateCamera with sub-pixel corner refinement
class CameraCalibrator {
  public:
    /// Default calibration parameters
    static constexpr int DEFAULT_PATTERN_WIDTH = 9;
    static constexpr int DEFAULT_PATTERN_HEIGHT = 6;
    static constexpr double DEFAULT_SQUARE_SIZE_M = 0.025; // 25mm
    static constexpr int DEFAULT_NUM_IMAGES = 15;

    /// Constructor with configurable parameters
    /// @param pattern_size Number of inner corners (width x height)
    /// @param square_size_m Real-world square size in meters
    /// @param num_images Number of images to capture for calibration
    CameraCalibrator(cv::Size pattern_size = cv::Size(DEFAULT_PATTERN_WIDTH,
                                                      DEFAULT_PATTERN_HEIGHT),
                     double square_size_m = DEFAULT_SQUARE_SIZE_M,
                     int num_images = DEFAULT_NUM_IMAGES);

    /// Run full interactive calibration for a camera
    /// @param device_path Video device path (e.g., "/dev/video0")
    /// @param mount Mount position for naming the output file
    /// @param output_dir Directory to save calibration YAML
    /// @return Calibration data if successful, nullopt on failure/cancel
    std::optional<CameraIntrinsics> runCalibration(const std::string &device_path, Mount mount,
                                                   const std::string &output_dir);

    /// Capture calibration images interactively
    /// Shows live preview with detected corners, user presses SPACE to capture
    /// @param device_path Video device path
    /// @return Vector of captured images with valid chessboard detections
    std::vector<cv::Mat> captureCalibrationImages(const std::string &device_path);

    /// Compute calibration from captured images
    /// @param images Vector of images with chessboard patterns
    /// @return Calibration data if successful, nullopt if failed
    std::optional<CameraIntrinsics> computeCalibration(const std::vector<cv::Mat> &images);

    /// Save calibration to YAML file
    /// @param calib Calibration data
    /// @param output_path Full path to output file
    /// @return true if saved successfully
    static bool saveCalibration(const CameraIntrinsics &calib, const std::string &output_path);

    /// Load calibration from YAML file
    /// @param input_path Path to calibration file
    /// @return Calibration data if file exists and is valid, nullopt otherwise
    static std::optional<CameraIntrinsics> loadCalibration(const std::string &input_path);

    /// Check if calibration file exists for a mount
    /// @param mount Camera mount position
    /// @param calib_dir Calibration directory
    /// @return true if calibration file exists
    static bool calibrationExists(Mount mount, const std::string &calib_dir);

    /// Get calibration file path for a mount
    /// @param mount Camera mount position
    /// @param calib_dir Calibration directory
    /// @return Full path to calibration file
    static std::string getCalibrationPath(Mount mount, const std::string &calib_dir);

  private:
    cv::Size pattern_size_;
    double square_size_m_;
    int num_images_;

    /// Find chessboard corners with sub-pixel refinement
    /// @param image Grayscale image
    /// @param corners Output corner positions
    /// @return true if chessboard found
    bool findCorners(const cv::Mat &image, std::vector<cv::Point2f> &corners) const;

    /// Generate object points (3D chessboard corners in world coordinates)
    std::vector<cv::Point3f> generateObjectPoints() const;

    /// Get current timestamp in ISO 8601 format
    static std::string getCurrentTimestamp();
};

} // namespace adas
