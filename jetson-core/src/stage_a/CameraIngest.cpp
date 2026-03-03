// File: src/stage_a/CameraIngest.cpp
// Direct USB camera capture implementation
// Based on test_scripts/cameraintake.py patterns
#include "adas/stage_a/CameraIngest.hpp"
#include "adas/recording/Recorder.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <iostream>
#include <numeric>

namespace adas {

CameraIngest::CameraIngest(Mount mount, const std::string &device_path,
                           SPSCQueue<CameraFrameData, 8> &queue,
                           const CameraConfig &config)
    : mount_(mount), device_path_(device_path), queue_(queue), config_(config) {
  frame_times_.fill(0);
}

CameraIngest::~CameraIngest() { stop(); }

void CameraIngest::start() {
  if (running_.load(std::memory_order_relaxed)) {
    return; // Already running
  }

  running_.store(true, std::memory_order_relaxed);
  thread_ = std::thread(&CameraIngest::run, this);
}

void CameraIngest::stop() {
  running_.store(false, std::memory_order_relaxed);
  if (thread_.joinable()) {
    thread_.join();
  }
  if (cap_ && cap_->isOpened()) {
    cap_->release();
  }
}

void CameraIngest::run() {
  std::cout << "[CameraIngest] Starting " << mountToString(mount_) << " on "
            << device_path_ << std::endl;

  // Open and configure camera
  if (!configureCamera()) {
    std::cerr << "[CameraIngest] Failed to configure " << mountToString(mount_)
              << std::endl;
    healthy_.store(false, std::memory_order_relaxed);
    running_.store(false, std::memory_order_relaxed);
    return;
  }

  healthy_.store(true, std::memory_order_relaxed);
  std::cout << "[CameraIngest] " << mountToString(mount_)
            << " configured successfully" << std::endl;

  // Warmup: discard first few frames (camera auto-exposure settling)
  for (int i = 0; i < 10 && running_.load(std::memory_order_relaxed); ++i) {
    cv::Mat dummy;
    cap_->read(dummy);
  }

  // Main capture loop
  while (running_.load(std::memory_order_relaxed)) {
    if (!captureFrame()) {
      // Brief sleep on error to avoid spinning
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::cout << "[CameraIngest] " << mountToString(mount_) << " stopped"
            << std::endl;
}

bool CameraIngest::configureCamera() {
// Open with V4L2 backend (Linux) or default (other platforms)
#ifdef __linux__
  cap_ = std::make_unique<cv::VideoCapture>(device_path_, cv::CAP_V4L2);
#else
  // Extract device number from path like "/dev/video0"
  int device_num = 0;
  if (device_path_.find("/dev/video") == 0) {
    device_num = std::stoi(device_path_.substr(10));
  } else {
    device_num = std::stoi(device_path_);
  }
  cap_ = std::make_unique<cv::VideoCapture>(device_num);
#endif

  if (!cap_->isOpened()) {
    std::cerr << "[CameraIngest] Cannot open " << device_path_ << std::endl;
    return false;
  }

  // CRITICAL: Force MJPEG codec BEFORE setting resolution.
  // Without MJPEG, camera defaults to uncompressed YUYV which at 1280x720@20fps
  // requires ~1.3 GB/s — instantly exhausts USB 2.0 bandwidth (480 Mbit/s).
  // This causes VIDIOC_STREAMON: No space left on device.
  if (config_.use_mjpeg) {
    cap_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  }

  // Set resolution based on mount (side cameras use lower res to save USB BW)
  if (mount_ == Mount::SideCamL || mount_ == Mount::SideCamR) {
    cap_->set(cv::CAP_PROP_FRAME_WIDTH, config_.side_width);
    cap_->set(cv::CAP_PROP_FRAME_HEIGHT, config_.side_height);
  } else {
    cap_->set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
    cap_->set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  }
  cap_->set(cv::CAP_PROP_FPS, config_.target_fps);

  // Verify actual negotiated settings
  double actual_width = cap_->get(cv::CAP_PROP_FRAME_WIDTH);
  double actual_height = cap_->get(cv::CAP_PROP_FRAME_HEIGHT);
  double actual_fps = cap_->get(cv::CAP_PROP_FPS);
  int actual_fourcc = static_cast<int>(cap_->get(cv::CAP_PROP_FOURCC));

  char codec[5] = {static_cast<char>(actual_fourcc & 0xFF),
                   static_cast<char>((actual_fourcc >> 8) & 0xFF),
                   static_cast<char>((actual_fourcc >> 16) & 0xFF),
                   static_cast<char>((actual_fourcc >> 24) & 0xFF), '\0'};

  std::cout << "[CameraIngest] " << mountToString(mount_)
            << " config: " << actual_width << "x" << actual_height << " @ "
            << actual_fps << " FPS, codec=" << codec << std::endl;

  // FATAL diagnostic: if MJPEG was requested but not granted, USB BW will fail.
  // Root cause: camera does not support MJPEG at this resolution, or V4L2
  // driver is overriding the format. Fix: lower resolution or use a camera
  // that supports MJPEG. VIDIOC_STREAMON: No space left on device = THIS.
  if (config_.use_mjpeg && std::string(codec) != "MJPG") {
    std::cerr << "[CameraIngest] *** FATAL WARNING *** "
              << mountToString(mount_) << "\n"
              << "  Camera REJECTED MJPEG and is using '" << codec
              << "' (uncompressed).\n"
              << "  This WILL cause VIDIOC_STREAMON: No space left on device "
                 "on USB 2.0.\n"
              << "  Fix: lower resolution in componentConfig.yaml (try 424x240 "
                 "for side cams)\n"
              << "       or verify camera supports MJPEG at " << actual_width
              << "x" << actual_height << ".\n";
  }

  return true;
}

bool CameraIngest::captureFrame() {
  cv::Mat frame;

  if (!cap_->read(frame)) {
    healthy_.store(false, std::memory_order_relaxed);
    return false;
  }

  // Timestamp IMMEDIATELY after read succeeds
  uint64_t t_ingest = Clock::now_ns();

  if (frame.empty()) {
    return false;
  }

  healthy_.store(true, std::memory_order_relaxed);

  // Build header
  uint32_t current_seq = seq_.fetch_add(1, std::memory_order_relaxed);
  Header header(t_ingest, mount_, current_seq, true);

  // Build frame data
  CameraFrameData frame_data;
  frame_data.h = header;
  frame_data.width = frame.cols;
  frame_data.height = frame.rows;
  frame_data.channels = frame.channels();

  // Copy pixel data (BGR format)
  size_t data_size = frame.total() * frame.elemSize();
  frame_data.data.resize(data_size);
  std::memcpy(frame_data.data.data(), frame.data, data_size);

  // Record before pushing to queue (if recording active)
  if (recorder_) {
    recorder_->recordCamera(frame_data, mount_);
  }

  // Push to queue
  if (!queue_.try_push(std::move(frame_data))) {
    // Queue full - frame dropped (counter in queue tracks this)
    return true; // Still "successful" capture
  }

  // Track frame times for FPS calculation
  frame_times_[frame_time_idx_] = t_ingest;
  frame_time_idx_ = (frame_time_idx_ + 1) % FPS_WINDOW;
  frames_captured_.fetch_add(1, std::memory_order_relaxed);

  return true;
}

CameraIngest::Stats CameraIngest::getStats() const {
  Stats stats{};
  stats.frames_captured = frames_captured_.load(std::memory_order_relaxed);
  stats.drops = queue_.drops();

  // Calculate FPS from frame times
  std::vector<double> deltas;
  deltas.reserve(FPS_WINDOW - 1);

  for (size_t i = 1; i < FPS_WINDOW; ++i) {
    size_t curr = (frame_time_idx_ + i) % FPS_WINDOW;
    size_t prev = (frame_time_idx_ + i - 1) % FPS_WINDOW;

    if (frame_times_[curr] > 0 && frame_times_[prev] > 0 &&
        frame_times_[curr] > frame_times_[prev]) {
      double delta_sec =
          Clock::ns_to_sec(frame_times_[curr] - frame_times_[prev]);
      if (delta_sec > 0) {
        deltas.push_back(1.0 / delta_sec);
      }
    }
  }

  if (!deltas.empty()) {
    stats.fps_avg =
        std::accumulate(deltas.begin(), deltas.end(), 0.0) / deltas.size();
    stats.fps_min = *std::min_element(deltas.begin(), deltas.end());
    stats.fps_max = *std::max_element(deltas.begin(), deltas.end());
  }

  return stats;
}

} // namespace adas
