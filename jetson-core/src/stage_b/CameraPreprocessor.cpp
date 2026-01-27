// File: src/stage_b/CameraPreprocessor.cpp
// Camera undistortion implementation
#include "adas/stage_b/CameraPreprocessor.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

#include <filesystem>
#include <iostream>

namespace adas {

CameraPreprocessor::CameraPreprocessor(Mount mount, const std::string &calib_dir) : mount_(mount) {

    // Build calibration file path
    std::string mount_name = mountToString(mount);
    std::string yaml_path = calib_dir + "/" + mount_name + "_calibration.yaml";

    if (!std::filesystem::exists(yaml_path)) {
        std::cerr << "[CameraPreprocessor] WARNING: Calibration not found for " << mount_name
                  << " at " << yaml_path << "\n";
        std::cerr << "[CameraPreprocessor] " << mount_name
                  << " will operate in pass-through mode (no undistortion)\n";
        return;
    }

    if (loadCalibration(yaml_path)) {
        computeUndistortMaps();
        calibrated_ = true;
        std::cout << "[CameraPreprocessor] " << mount_name
                  << " calibration loaded (RMS: " << intrinsics_.rms_error << ")\n";
    }
}

bool CameraPreprocessor::loadCalibration(const std::string &yaml_path) {
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[CameraPreprocessor] ERROR: Cannot open " << yaml_path << "\n";
        return false;
    }

    try {
        fs["camera_matrix"] >> intrinsics_.camera_matrix;
        fs["distortion_coefficients"] >> intrinsics_.dist_coeffs;
        intrinsics_.image_width = static_cast<int>(fs["image_width"].real());
        intrinsics_.image_height = static_cast<int>(fs["image_height"].real());
        intrinsics_.rms_error = fs["rms_error"].real();

        // Validate loaded data
        if (intrinsics_.camera_matrix.empty() || intrinsics_.dist_coeffs.empty()) {
            std::cerr << "[CameraPreprocessor] ERROR: Invalid calibration data in " << yaml_path
                      << "\n";
            return false;
        }

        if (intrinsics_.image_width <= 0 || intrinsics_.image_height <= 0) {
            std::cerr << "[CameraPreprocessor] ERROR: Invalid image dimensions in " << yaml_path
                      << "\n";
            return false;
        }

        intrinsics_.valid = true;
        fs.release();
        return true;

    } catch (const cv::Exception &e) {
        std::cerr << "[CameraPreprocessor] ERROR parsing calibration: " << e.what() << "\n";
        return false;
    }
}

void CameraPreprocessor::computeUndistortMaps() {
    cv::Size image_size(intrinsics_.image_width, intrinsics_.image_height);

    // Get optimal new camera matrix with valid ROI
    new_camera_matrix_ = cv::getOptimalNewCameraMatrix(
        intrinsics_.camera_matrix, intrinsics_.dist_coeffs, image_size,
        1.0, // alpha = 1.0 keeps all pixels (some black borders)
        image_size, &roi_);

    // Precompute undistortion maps for efficient cv::remap
    cv::initUndistortRectifyMap(intrinsics_.camera_matrix, intrinsics_.dist_coeffs,
                                cv::Mat(), // No rectification
                                new_camera_matrix_, image_size, CV_32FC1, map_x_, map_y_);

    std::cout << "[CameraPreprocessor] Undistort maps computed. ROI: " << roi_.width << "x"
              << roi_.height << "\n";
}

cv::Mat CameraPreprocessor::process(const cv::Mat &input, bool crop_to_roi) {
    if (!calibrated_ || input.empty()) {
        return input.clone(); // Pass-through if not calibrated
    }

    cv::Mat undistorted;
    cv::remap(input, undistorted, map_x_, map_y_, cv::INTER_LINEAR);

    if (crop_to_roi && roi_.width > 0 && roi_.height > 0) {
        return undistorted(roi_).clone();
    }

    return undistorted;
}

CameraFrameData CameraPreprocessor::process(const CameraFrameData &input, bool crop_to_roi) {
    CameraFrameData output;
    output.h = input.h;

    if (!calibrated_ || input.data.empty()) {
        output = input; // Pass-through
        return output;
    }

    // Convert vector<uint8_t> to cv::Mat
    cv::Mat frame(input.height, input.width, CV_8UC3, const_cast<uint8_t *>(input.data.data()));

    // Apply undistortion
    cv::Mat undistorted = process(frame, crop_to_roi);

    // Update output dimensions and data
    output.width = undistorted.cols;
    output.height = undistorted.rows;
    output.channels = undistorted.channels();

    // Copy to output vector
    size_t data_size = undistorted.total() * undistorted.elemSize();
    output.data.resize(data_size);
    std::memcpy(output.data.data(), undistorted.data, data_size);

    return output;
}

} // namespace adas
