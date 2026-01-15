// File: src/stage_a/CameraCalibrator.cpp
// Camera intrinsic calibration implementation
#include "adas/stage_a/CameraCalibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace adas {

CameraCalibrator::CameraCalibrator(cv::Size pattern_size, double square_size_m, int num_images)
    : pattern_size_(pattern_size), square_size_m_(square_size_m), num_images_(num_images) {}

std::optional<CameraIntrinsics> CameraCalibrator::runCalibration(const std::string &device_path,
                                                                 Mount mount,
                                                                 const std::string &output_dir) {
    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "          Camera Calibration: " << mountToString(mount) << "\n";
    std::cout << "==============================================================\n";
    std::cout << "  Device: " << device_path << "\n";
    std::cout << "  Pattern: " << pattern_size_.width << "x" << pattern_size_.height
              << " inner corners\n";
    std::cout << "  Square size: " << (square_size_m_ * 1000) << " mm\n";
    std::cout << "  Images to capture: " << num_images_ << "\n";
    std::cout << "==============================================================\n\n";

    std::cout << "  Instructions:\n";
    std::cout << "  - Hold the chessboard pattern in front of the camera\n";
    std::cout << "  - Move it to different positions and angles\n";
    std::cout << "  - Green overlay = pattern detected\n";
    std::cout << "  - Press SPACE to capture when corners are shown\n";
    std::cout << "  - Press ESC to cancel calibration\n";
    std::cout << "  - Press Q when done (minimum 10 images recommended)\n\n";

    // Capture images
    auto images = captureCalibrationImages(device_path);

    if (images.empty()) {
        std::cerr << "  [ERROR] No valid calibration images captured.\n";
        return std::nullopt;
    }

    if (images.size() < 10) {
        std::cerr << "  [WARNING] Only " << images.size()
                  << " images captured. Recommend at least 10 for good calibration.\n";
        std::cout << "  Continue anyway? (y/n): ";
        char choice;
        std::cin >> choice;
        if (choice != 'y' && choice != 'Y') {
            return std::nullopt;
        }
    }

    std::cout << "\n  Computing calibration from " << images.size() << " images...\n";

    // Compute calibration
    auto calib = computeCalibration(images);
    if (!calib.has_value()) {
        std::cerr << "  [ERROR] Calibration computation failed.\n";
        return std::nullopt;
    }

    // Save calibration
    std::filesystem::create_directories(output_dir);
    std::string output_path = getCalibrationPath(mount, output_dir);

    if (saveCalibration(calib.value(), output_path)) {
        std::cout << "  [SUCCESS] Calibration saved to: " << output_path << "\n";
        std::cout << "  RMS reprojection error: " << calib->rms_error << " pixels\n";

        if (calib->rms_error < 0.5) {
            std::cout << "  Quality: EXCELLENT\n";
        } else if (calib->rms_error < 1.0) {
            std::cout << "  Quality: GOOD\n";
        } else if (calib->rms_error < 2.0) {
            std::cout << "  Quality: ACCEPTABLE\n";
        } else {
            std::cout << "  Quality: POOR - consider recalibrating\n";
        }
    } else {
        std::cerr << "  [ERROR] Failed to save calibration.\n";
        return std::nullopt;
    }

    return calib;
}

std::vector<cv::Mat> CameraCalibrator::captureCalibrationImages(const std::string &device_path) {
    std::vector<cv::Mat> captured_images;

#ifdef __linux__
    cv::VideoCapture cap(device_path, cv::CAP_V4L2);
#else
    int device_num = std::stoi(device_path);
    cv::VideoCapture cap(device_num);
#endif

    if (!cap.isOpened()) {
        std::cerr << "  [ERROR] Cannot open video device: " << device_path << "\n";
        return captured_images;
    }

    // Configure camera
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    std::string window_name = "Calibration - SPACE to capture, Q when done, ESC to cancel";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    int captured_count = 0;

    while (true) {
        cv::Mat frame, gray, display;
        if (!cap.read(frame)) {
            std::cerr << "  [ERROR] Failed to read frame\n";
            break;
        }

        frame.copyTo(display);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Try to find chessboard corners
        std::vector<cv::Point2f> corners;
        bool found = findCorners(gray, corners);

        if (found) {
            // Draw detected corners
            cv::drawChessboardCorners(display, pattern_size_, corners, found);

            // Add green border to indicate pattern detected
            cv::rectangle(display, cv::Point(0, 0), cv::Point(display.cols - 1, display.rows - 1),
                          cv::Scalar(0, 255, 0), 5);
        }

        // Draw status text
        std::string status =
            "Captured: " + std::to_string(captured_count) + "/" + std::to_string(num_images_);
        if (found) {
            status += " | Pattern DETECTED - Press SPACE";
        } else {
            status += " | Looking for pattern...";
        }

        cv::putText(display, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 0), 2);

        cv::imshow(window_name, display);

        int key = cv::waitKey(30);

        if (key == 27) { // ESC
            std::cout << "  Calibration cancelled by user.\n";
            captured_images.clear();
            break;
        } else if (key == 'q' || key == 'Q') {
            std::cout << "  Capture complete.\n";
            break;
        } else if (key == ' ' && found) { // SPACE
            // Save the grayscale image for calibration
            captured_images.push_back(gray.clone());
            captured_count++;
            std::cout << "  Captured image " << captured_count << "/" << num_images_ << "\n";

            // Flash feedback
            cv::Mat white(display.size(), display.type(), cv::Scalar(255, 255, 255));
            cv::imshow(window_name, white);
            cv::waitKey(100);

            if (captured_count >= num_images_) {
                std::cout << "  Target number of images reached.\n";
                break;
            }
        }
    }

    cv::destroyWindow(window_name);
    cap.release();

    return captured_images;
}

bool CameraCalibrator::findCorners(const cv::Mat &image, std::vector<cv::Point2f> &corners) const {
    // Find chessboard corners
    int flags =
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;

    bool found = cv::findChessboardCorners(image, pattern_size_, corners, flags);

    if (found) {
        // Sub-pixel refinement
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
        cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
    }

    return found;
}

std::vector<cv::Point3f> CameraCalibrator::generateObjectPoints() const {
    std::vector<cv::Point3f> points;
    for (int i = 0; i < pattern_size_.height; ++i) {
        for (int j = 0; j < pattern_size_.width; ++j) {
            points.emplace_back(j * square_size_m_, i * square_size_m_, 0.0f);
        }
    }
    return points;
}

std::optional<CameraIntrinsics>
CameraCalibrator::computeCalibration(const std::vector<cv::Mat> &images) {

    if (images.empty()) {
        return std::nullopt;
    }

    // Get image size from first image
    cv::Size image_size = images[0].size();

    // Prepare object points and image points
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<cv::Point3f> object_template = generateObjectPoints();

    for (const auto &image : images) {
        std::vector<cv::Point2f> corners;
        if (findCorners(image, corners)) {
            object_points.push_back(object_template);
            image_points.push_back(corners);
        }
    }

    if (object_points.size() < 5) {
        std::cerr << "  [ERROR] Need at least 5 valid images for calibration.\n";
        return std::nullopt;
    }

    std::cout << "  Using " << object_points.size() << " images for calibration...\n";

    // Calibrate camera
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(object_points, image_points, image_size, camera_matrix,
                                     dist_coeffs, rvecs, tvecs);

    // Build result
    CameraIntrinsics calib;
    calib.camera_matrix = camera_matrix.clone();
    calib.distortion_coefficients = dist_coeffs.clone();
    calib.image_width = image_size.width;
    calib.image_height = image_size.height;
    calib.rms_error = rms;
    calib.calibrated_at = getCurrentTimestamp();
    calib.pattern_size = pattern_size_;
    calib.square_size_m = square_size_m_;

    return calib;
}

bool CameraCalibrator::saveCalibration(const CameraIntrinsics &calib,
                                       const std::string &output_path) {
    cv::FileStorage fs(output_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }

    fs << "image_width" << calib.image_width;
    fs << "image_height" << calib.image_height;
    fs << "camera_matrix" << calib.camera_matrix;
    fs << "distortion_coefficients" << calib.distortion_coefficients;
    fs << "rms_error" << calib.rms_error;
    fs << "calibrated_at" << calib.calibrated_at;
    fs << "pattern_width" << calib.pattern_size.width;
    fs << "pattern_height" << calib.pattern_size.height;
    fs << "square_size_m" << calib.square_size_m;

    fs.release();
    return true;
}

std::optional<CameraIntrinsics> CameraCalibrator::loadCalibration(const std::string &input_path) {
    if (!std::filesystem::exists(input_path)) {
        return std::nullopt;
    }

    cv::FileStorage fs(input_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return std::nullopt;
    }

    CameraIntrinsics calib;
    fs["image_width"] >> calib.image_width;
    fs["image_height"] >> calib.image_height;
    fs["camera_matrix"] >> calib.camera_matrix;
    fs["distortion_coefficients"] >> calib.distortion_coefficients;
    fs["rms_error"] >> calib.rms_error;
    fs["calibrated_at"] >> calib.calibrated_at;

    int pw, ph;
    fs["pattern_width"] >> pw;
    fs["pattern_height"] >> ph;
    calib.pattern_size = cv::Size(pw, ph);

    fs["square_size_m"] >> calib.square_size_m;

    fs.release();

    // Validate required fields
    if (calib.camera_matrix.empty() || calib.distortion_coefficients.empty()) {
        return std::nullopt;
    }

    return calib;
}

bool CameraCalibrator::calibrationExists(Mount mount, const std::string &calib_dir) {
    return std::filesystem::exists(getCalibrationPath(mount, calib_dir));
}

std::string CameraCalibrator::getCalibrationPath(Mount mount, const std::string &calib_dir) {
    return calib_dir + "/" + mountToString(mount) + "_calibration.yaml";
}

std::string CameraCalibrator::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::gmtime(&time_t_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace adas
