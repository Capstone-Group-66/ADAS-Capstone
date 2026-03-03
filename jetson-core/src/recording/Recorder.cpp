// File: src/recording/Recorder.cpp
// Record sensor data to .adasrec binary file
#include "adas/recording/Recorder.hpp"

#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

namespace adas {

Recorder::~Recorder() { stop(); }

bool Recorder::start(const std::string &output_dir) {
  if (recording_.load(std::memory_order_relaxed)) {
    std::cerr << "[Recorder] Already recording\n";
    return false;
  }

  // Create output directory if needed
  std::filesystem::create_directories(output_dir);

  // Generate filename: run_YYYYMMDD_HHMMSS.adasrec
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf;
#ifdef _WIN32
  localtime_s(&tm_buf, &time_t);
#else
  localtime_r(&time_t, &tm_buf);
#endif
  char time_str[32];
  std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &tm_buf);

  file_path_ = output_dir + "/run_" + std::string(time_str) + ".adasrec";

  // Open file
  out_file_.open(file_path_, std::ios::binary | std::ios::trunc);
  if (!out_file_.is_open()) {
    std::cerr << "[Recorder] Failed to open: " << file_path_ << "\n";
    return false;
  }

  // Write file header (start_time_ns will be updated with first event)
  start_time_ns_ = Clock::now_ns();
  AdasRecFileHeader file_header;
  file_header.start_time_ns = start_time_ns_;
  file_header.sensor_mask = 0xFFFF; // All sensors
  out_file_.write(reinterpret_cast<const char *>(&file_header),
                  sizeof(file_header));

  // Reset counters
  event_count_.store(0, std::memory_order_relaxed);
  bytes_written_.store(sizeof(AdasRecFileHeader), std::memory_order_relaxed);

  // Start writer thread
  writer_stop_.store(false, std::memory_order_relaxed);
  recording_.store(true, std::memory_order_relaxed);
  writer_thread_ = std::thread(&Recorder::writerLoop, this);

  std::cout << "[Recorder] Recording to: " << file_path_ << "\n";
  return true;
}

void Recorder::stop() {
  if (!recording_.load(std::memory_order_relaxed)) {
    return;
  }

  recording_.store(false, std::memory_order_relaxed);
  writer_stop_.store(true, std::memory_order_relaxed);

  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  // Flush remaining events
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    for (auto &event : write_buffer_) {
      RecEventHeader hdr;
      hdr.timestamp_ns = event.timestamp_ns;
      hdr.type = static_cast<uint8_t>(event.type);
      hdr.payload_size = static_cast<uint32_t>(event.payload.size());
      out_file_.write(reinterpret_cast<const char *>(&hdr), sizeof(hdr));
      out_file_.write(reinterpret_cast<const char *>(event.payload.data()),
                      event.payload.size());
    }
    write_buffer_.clear();
  }

  out_file_.flush();
  out_file_.close();

  std::cout << "[Recorder] Stopped. " << event_count_.load() << " events, "
            << (bytes_written_.load() / (1024 * 1024)) << " MB written to "
            << file_path_ << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
//                       RECORDING METHODS (producer threads)
// ═══════════════════════════════════════════════════════════════════════════════

void Recorder::recordCamera(const CameraFrameData &frame, Mount mount) {
  if (!recording_.load(std::memory_order_relaxed))
    return;

  // JPEG encode the frame on the caller thread
  // (for USB cameras, frame.data contains raw BGR pixels)
  std::vector<uint8_t> jpeg_buf;
  cv::Mat mat;

  if (!frame.frame.empty()) {
    mat = frame.frame;
  } else if (!frame.data.empty()) {
    mat = cv::Mat(frame.height, frame.width, CV_8UC3,
                  const_cast<uint8_t *>(frame.data.data()));
  }

  if (mat.empty())
    return;

  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
  cv::imencode(".jpg", mat, jpeg_buf, params);

  // Build payload: mount(1B) + width(2B) + height(2B) + jpeg_bytes
  RecordEvent event;
  event.type = mountToCameraEvent(mount);
  event.timestamp_ns = frame.h.t_ingest_ns;

  size_t header_size = 1 + 2 + 2; // mount + w + h
  event.payload.resize(header_size + jpeg_buf.size());

  event.payload[0] = static_cast<uint8_t>(mount);
  uint16_t w = static_cast<uint16_t>(frame.width);
  uint16_t h = static_cast<uint16_t>(frame.height);
  std::memcpy(&event.payload[1], &w, 2);
  std::memcpy(&event.payload[3], &h, 2);
  std::memcpy(&event.payload[5], jpeg_buf.data(), jpeg_buf.size());

  enqueue(std::move(event));
}

void Recorder::recordCameraJpeg(const uint8_t *jpeg, size_t len,
                                uint64_t timestamp_ns, Mount mount,
                                uint16_t width, uint16_t height) {
  if (!recording_.load(std::memory_order_relaxed))
    return;

  RecordEvent event;
  event.type = mountToCameraEvent(mount);
  event.timestamp_ns = timestamp_ns;

  size_t header_size = 1 + 2 + 2;
  event.payload.resize(header_size + len);

  event.payload[0] = static_cast<uint8_t>(mount);
  std::memcpy(&event.payload[1], &width, 2);
  std::memcpy(&event.payload[3], &height, 2);
  std::memcpy(&event.payload[5], jpeg, len);

  enqueue(std::move(event));
}

void Recorder::recordRadar(const RadarTargets &targets) {
  if (!recording_.load(std::memory_order_relaxed))
    return;

  RecordEvent event;
  event.type = mountToRadarEvent(targets.h.mount);
  event.timestamp_ns = targets.h.t_ingest_ns;

  // Payload: n_targets(1B) + [range(4B), az(4B), vel(4B), rcs(4B),
  //                            sigma_r(4B), sigma_az(4B), sigma_v(4B)] × N
  uint8_t n = static_cast<uint8_t>(
      std::min(targets.targets.size(), static_cast<size_t>(255)));
  size_t per_target = 7 * sizeof(float);
  event.payload.resize(1 + n * per_target);

  event.payload[0] = n;
  size_t offset = 1;
  for (uint8_t i = 0; i < n; ++i) {
    const auto &t = targets.targets[i];
    std::memcpy(&event.payload[offset + 0], &t.range_m, 4);
    std::memcpy(&event.payload[offset + 4], &t.azimuth_rad, 4);
    std::memcpy(&event.payload[offset + 8], &t.radial_vel_mps, 4);
    std::memcpy(&event.payload[offset + 12], &t.rcs_db, 4);
    std::memcpy(&event.payload[offset + 16], &t.sigma_r, 4);
    std::memcpy(&event.payload[offset + 20], &t.sigma_az, 4);
    std::memcpy(&event.payload[offset + 24], &t.sigma_v, 4);
    offset += per_target;
  }

  enqueue(std::move(event));
}

void Recorder::recordIMU(const ImuSample &sample) {
  if (!recording_.load(std::memory_order_relaxed))
    return;

  RecordEvent event;
  event.type = RecEventType::IMU;
  event.timestamp_ns = sample.t_capture;

  // Fixed 60-byte payload
  event.payload.resize(60);
  size_t off = 0;

  // accel[3] (12B)
  std::memcpy(&event.payload[off], sample.accel.data(), 12);
  off += 12;
  // gyro[3] (12B)
  std::memcpy(&event.payload[off], sample.gyro.data(), 12);
  off += 12;
  // mag[3] (12B)
  std::memcpy(&event.payload[off], sample.mag.data(), 12);
  off += 12;
  // quat[4] (16B)
  std::memcpy(&event.payload[off], sample.quat.data(), 16);
  off += 16;
  // temperature (4B)
  std::memcpy(&event.payload[off], &sample.temperature, 4);
  off += 4;
  // calibration_status (1B)
  event.payload[off] = sample.calibration_status;
  off += 1;
  // padding (1B) to reach 60
  event.payload[off] = 0;

  enqueue(std::move(event));
}

void Recorder::recordGPS(float speed_mps, uint64_t ts_ms) {
  if (!recording_.load(std::memory_order_relaxed))
    return;

  RecordEvent event;
  event.type = RecEventType::GPS;
  event.timestamp_ns = Clock::now_ns();

  // 32-byte payload: speed_mps(4B) + ts_ms(8B) + reserved(20B)
  event.payload.resize(32, 0);
  std::memcpy(&event.payload[0], &speed_mps, 4);
  std::memcpy(&event.payload[4], &ts_ms, 8);

  enqueue(std::move(event));
}

// ═══════════════════════════════════════════════════════════════════════════════
//                           INTERNAL METHODS
// ═══════════════════════════════════════════════════════════════════════════════

void Recorder::enqueue(RecordEvent &&event) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  write_buffer_.push_back(std::move(event));
}

void Recorder::writerLoop() {
  std::cout << "[Recorder] Writer thread started\n";

  while (!writer_stop_.load(std::memory_order_relaxed)) {
    // Swap buffers under lock (minimizes lock hold time)
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      std::swap(write_buffer_, drain_buffer_);
    }

    if (drain_buffer_.empty()) {
      // No events to write — sleep briefly
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // Write all drained events to disk
    for (const auto &event : drain_buffer_) {
      RecEventHeader hdr;
      hdr.timestamp_ns = event.timestamp_ns;
      hdr.type = static_cast<uint8_t>(event.type);
      hdr.payload_size = static_cast<uint32_t>(event.payload.size());

      out_file_.write(reinterpret_cast<const char *>(&hdr), sizeof(hdr));
      out_file_.write(reinterpret_cast<const char *>(event.payload.data()),
                      event.payload.size());

      bytes_written_.fetch_add(sizeof(hdr) + event.payload.size(),
                               std::memory_order_relaxed);
      event_count_.fetch_add(1, std::memory_order_relaxed);
    }

    out_file_.flush();
    drain_buffer_.clear();
  }

  std::cout << "[Recorder] Writer thread stopped\n";
}

} // namespace adas
