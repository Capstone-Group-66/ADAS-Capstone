// File: src/recording/ReplayEngine.cpp
// Replay .adasrec files into pipeline SPSC queues
#include "adas/recording/ReplayEngine.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>

namespace adas {

namespace {

uint64_t scaledReplayOffsetNs(uint64_t event_offset_ns, float speed) {
  if (speed <= 0.0f) {
    return event_offset_ns;
  }
  return static_cast<uint64_t>(event_offset_ns / speed);
}

} // namespace

ReplayEngine::~ReplayEngine() { stop(); }

bool ReplayEngine::load(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "[ReplayEngine] Cannot open: " << path << "\n";
    return false;
  }

  // Read file header
  in.read(reinterpret_cast<char *>(&file_header_), sizeof(file_header_));
  if (std::strncmp(file_header_.magic, "AREC", 4) != 0) {
    std::cerr << "[ReplayEngine] Invalid magic bytes in: " << path << "\n";
    return false;
  }
  if (file_header_.version != 1 && file_header_.version != 3) {
    std::cerr << "[ReplayEngine] Unsupported version: " << file_header_.version
              << "\n";
    return false;
  }
  if (file_header_.version == 1) {
    std::cout << "[ReplayEngine] WARNING: Legacy v1 recording loaded. Front "
                 "DeepStream detections and RCW state were not recorded, so "
                 "replay cannot fully reproduce the current architecture.\n";
  }

  // Read all events
  events_.clear();
  while (in.good() && !in.eof()) {
    RecEventHeader hdr;
    in.read(reinterpret_cast<char *>(&hdr), sizeof(hdr));
    if (in.gcount() < static_cast<std::streamsize>(sizeof(hdr))) {
      break; // EOF or partial read
    }

    RecordEvent event;
    event.type = static_cast<RecEventType>(hdr.type);
    event.timestamp_ns = hdr.timestamp_ns;
    event.payload.resize(hdr.payload_size);

    if (hdr.payload_size > 0) {
      in.read(reinterpret_cast<char *>(event.payload.data()), hdr.payload_size);
      if (in.gcount() < static_cast<std::streamsize>(hdr.payload_size)) {
        std::cerr << "[ReplayEngine] Truncated event at index "
                  << events_.size() << "\n";
        break;
      }
    }

    events_.push_back(std::move(event));
  }

  // Sort by timestamp (should already be ordered, but ensure correctness)
  std::sort(events_.begin(), events_.end(),
            [](const RecordEvent &a, const RecordEvent &b) {
              return a.timestamp_ns < b.timestamp_ns;
            });

  std::cout << "[ReplayEngine] Loaded " << events_.size() << " events from "
            << path << "\n";
  if (!events_.empty()) {
    uint64_t dur_ns =
        events_.back().timestamp_ns - events_.front().timestamp_ns;
    double dur_s = dur_ns / 1e9;
    std::cout << "[ReplayEngine] Duration: " << dur_s << "s\n";
  }

  return !events_.empty();
}

void ReplayEngine::start() {
  if (events_.empty()) {
    std::cerr << "[ReplayEngine] No events loaded\n";
    return;
  }

  current_index_.store(0, std::memory_order_relaxed);
  finished_.store(false, std::memory_order_relaxed);
  running_.store(true, std::memory_order_relaxed);

  replay_thread_ = std::thread(&ReplayEngine::replayLoop, this);
  std::cout << "[ReplayEngine] Replay started (speed=" << speed_ << "x)\n";
}

void ReplayEngine::stop() {
  if (!running_.load(std::memory_order_relaxed)) {
    return;
  }
  running_.store(false, std::memory_order_relaxed);
  if (replay_thread_.joinable()) {
    replay_thread_.join();
  }
  std::cout << "[ReplayEngine] Stopped at event "
            << current_index_.load(std::memory_order_relaxed) << "/"
            << events_.size() << "\n";
}

float ReplayEngine::getProgress() const {
  if (events_.empty())
    return 0.0f;
  return static_cast<float>(current_index_.load(std::memory_order_relaxed)) /
         static_cast<float>(events_.size());
}

uint64_t ReplayEngine::getDurationNs() const {
  if (events_.size() < 2)
    return 0;
  return events_.back().timestamp_ns - events_.front().timestamp_ns;
}

void ReplayEngine::setFrontDetQueue(SPSCQueue<DetBatch, 8> *queue) {
  front_det_queue_ = queue;
}

void ReplayEngine::setRcwQueue(SPSCQueue<RcwState, 16> *queue) {
  rcw_queue_ = queue;
}

void ReplayEngine::setRadarQueue(Mount mount,
                                 SPSCQueue<RadarTargets, 8> *queue) {
  switch (mount) {
  case Mount::FrontRadar:
    radar_front_queue_ = queue;
    break;
  case Mount::RearCornerRadarL:
    radar_rear_l_queue_ = queue;
    break;
  case Mount::RearCornerRadarR:
    radar_rear_r_queue_ = queue;
    break;
  default:
    break;
  }
}

void ReplayEngine::setIMUQueue(SPSCQueue<ImuSample, 32> *queue) {
  imu_queue_ = queue;
}

void ReplayEngine::setGpsCallback(std::function<void(float, uint64_t)> cb) {
  gps_callback_ = std::move(cb);
}

// ═══════════════════════════════════════════════════════════════════════════════
//                              REPLAY LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void ReplayEngine::replayLoop() {
  if (events_.empty())
    return;

  first_event_ts_ = events_.front().timestamp_ns;
  const auto wall_start = std::chrono::steady_clock::now();
  replay_start_time_ns_ = Clock::now_ns();
  const bool fast_mode = (speed_ <= 0.0f);

  for (size_t i = 0;
       i < events_.size() && running_.load(std::memory_order_relaxed); ++i) {
    current_index_.store(i, std::memory_order_relaxed);
    const auto &event = events_[i];

    if (!fast_mode) {
      // Compute wall-clock target for this event
      uint64_t event_offset_ns = event.timestamp_ns - first_event_ts_;
      double scaled_offset_ns = event_offset_ns / static_cast<double>(speed_);

      auto target_time =
          wall_start +
          std::chrono::nanoseconds(static_cast<int64_t>(scaled_offset_ns));

      // Coarse sleep: sleep until ~500µs before target
      auto coarse_target = target_time - std::chrono::microseconds(500);
      auto now = std::chrono::steady_clock::now();
      if (coarse_target > now) {
        std::this_thread::sleep_until(coarse_target);
      }

      // Fine spin: busy-wait for sub-millisecond precision
      while (std::chrono::steady_clock::now() < target_time &&
             running_.load(std::memory_order_relaxed)) {
        // spin
      }
    }

    // Dispatch the event to the appropriate queue
    dispatchEvent(event);
  }

  finished_.store(true, std::memory_order_relaxed);
  std::cout << "[ReplayEngine] Replay complete (" << events_.size()
            << " events)\n";
}

void ReplayEngine::dispatchEvent(const RecordEvent &event) {
  switch (event.type) {
  case RecEventType::CameraFront:
  case RecEventType::CameraSideL:
  case RecEventType::CameraSideR:
  case RecEventType::CameraRear:
    dispatchCamera(event);
    break;
  case RecEventType::FrontDetBatch:
    dispatchFrontDetBatch(event);
    break;
  case RecEventType::RCWState:
    dispatchRcwState(event);
    break;
  case RecEventType::RadarFront:
  case RecEventType::RadarRearL:
  case RecEventType::RadarRearR:
    dispatchRadar(event);
    break;
  case RecEventType::IMU:
    dispatchIMU(event);
    break;
  case RecEventType::GPS:
    dispatchGPS(event);
    break;
  default:
    // Unknown event type — skip (forward compatibility)
    break;
  }
}

void ReplayEngine::dispatchCamera(const RecordEvent &event) {
  (void)event;
  // Legacy v1 raw-camera events are intentionally ignored. The current
  // architecture replays front DeepStream detections and compact RCW state,
  // not raw camera frames.
}

void ReplayEngine::dispatchFrontDetBatch(const RecordEvent &event) {
  if (event.payload.size() < sizeof(RecFrontDetBatchHeader) ||
      !front_det_queue_) {
    return;
  }

  RecFrontDetBatchHeader batch_header;
  std::memcpy(&batch_header, event.payload.data(), sizeof(batch_header));

  const size_t expected_size =
      sizeof(batch_header) +
      static_cast<size_t>(batch_header.num_detections) * sizeof(RecFrontDet);
  if (event.payload.size() < expected_size) {
    return;
  }

  const uint64_t event_offset_ns = event.timestamp_ns - first_event_ts_;
  const uint64_t synthetic_ts_ns =
      replay_start_time_ns_ + scaledReplayOffsetNs(event_offset_ns, speed_);

  DetBatch batch;
  batch.h.mount = Mount::FrontCam;
  batch.h.t_ingest_ns = synthetic_ts_ns;
  batch.h.t_device_ns = synthetic_ts_ns;
  batch.h.seq = 0;
  batch.h.healthy = true;
  batch.inference_time_us = batch_header.inference_time_us;

  size_t offset = sizeof(batch_header);
  for (uint32_t i = 0; i < batch_header.num_detections; ++i) {
    RecFrontDet rec_det;
    std::memcpy(&rec_det, event.payload.data() + offset, sizeof(rec_det));
    offset += sizeof(rec_det);

    Det det;
    det.box_px = cv::Rect2f(rec_det.x, rec_det.y, rec_det.w, rec_det.h);
    det.centroid = cv::Point2f(rec_det.centroid_x, rec_det.centroid_y);
    det.cls = rec_det.cls;
    det.score = rec_det.score;
    det.object_id = rec_det.object_id;
    det.setSignLabel(rec_det.sign_label);
    batch.dets.push_back(det);
  }

  front_det_queue_->try_push(std::move(batch));
}

void ReplayEngine::dispatchRcwState(const RecordEvent &event) {
  if (event.payload.size() < sizeof(RecRcwStatePayload) || !rcw_queue_) {
    return;
  }

  RecRcwStatePayload payload{};
  std::memcpy(&payload, event.payload.data(), sizeof(payload));

  const uint64_t event_offset_ns = event.timestamp_ns - first_event_ts_;
  const uint64_t synthetic_ts_ns =
      replay_start_time_ns_ + scaledReplayOffsetNs(event_offset_ns, speed_);

  RcwState rcw_state;
  rcw_state.h.mount = Mount::RearCam;
  rcw_state.h.t_ingest_ns = synthetic_ts_ns;
  rcw_state.h.t_device_ns = synthetic_ts_ns;
  rcw_state.h.seq = 0;
  rcw_state.h.healthy = true;
  rcw_state.alert = payload.alert;
  rcw_state.status = payload.status;
  rcw_queue_->try_push(std::move(rcw_state));
}

void ReplayEngine::dispatchRadar(const RecordEvent &event) {
  if (event.payload.empty())
    return;

  uint8_t n = event.payload[0];
  size_t per_target = 7 * sizeof(float);

  RadarTargets targets;
  // Map event type back to mount
  switch (event.type) {
  case RecEventType::RadarFront:
    targets.h.mount = Mount::FrontRadar;
    break;
  case RecEventType::RadarRearL:
    targets.h.mount = Mount::RearCornerRadarL;
    break;
  case RecEventType::RadarRearR:
    targets.h.mount = Mount::RearCornerRadarR;
    break;
  default:
    break;
  }

  // Calculate synthetic timestamp
  uint64_t event_offset_ns = event.timestamp_ns - first_event_ts_;
  uint64_t synthetic_ts_ns =
      replay_start_time_ns_ + scaledReplayOffsetNs(event_offset_ns, speed_);

  targets.h.t_ingest_ns = synthetic_ts_ns;
  targets.h.t_device_ns = synthetic_ts_ns;

  size_t offset = 1;
  for (uint8_t i = 0; i < n && offset + per_target <= event.payload.size();
       ++i) {
    RadarTarget t;
    std::memcpy(&t.range_m, &event.payload[offset + 0], 4);
    std::memcpy(&t.azimuth_rad, &event.payload[offset + 4], 4);
    std::memcpy(&t.radial_vel_mps, &event.payload[offset + 8], 4);
    std::memcpy(&t.rcs_db, &event.payload[offset + 12], 4);
    std::memcpy(&t.sigma_r, &event.payload[offset + 16], 4);
    std::memcpy(&t.sigma_az, &event.payload[offset + 20], 4);
    std::memcpy(&t.sigma_v, &event.payload[offset + 24], 4);
    targets.targets.push_back(t);
    offset += per_target;
  }

  // Push to appropriate queue
  SPSCQueue<RadarTargets, 8> *queue = nullptr;
  switch (event.type) {
  case RecEventType::RadarFront:
    queue = radar_front_queue_;
    break;
  case RecEventType::RadarRearL:
    queue = radar_rear_l_queue_;
    break;
  case RecEventType::RadarRearR:
    queue = radar_rear_r_queue_;
    break;
  default:
    break;
  }

  if (queue) {
    queue->try_push(std::move(targets));
  }
}

void ReplayEngine::dispatchIMU(const RecordEvent &event) {
  if (event.payload.size() < 57 || !imu_queue_) // minimum: 12+12+12+16+4+1 = 57
    return;

  // Calculate synthetic timestamp
  uint64_t event_offset_ns = event.timestamp_ns - first_event_ts_;
  uint64_t synthetic_ts_ns =
      replay_start_time_ns_ + scaledReplayOffsetNs(event_offset_ns, speed_);

  ImuSample sample;
  sample.t_capture = synthetic_ts_ns;

  size_t off = 0;
  std::memcpy(sample.accel.data(), &event.payload[off], 12);
  off += 12;
  std::memcpy(sample.gyro.data(), &event.payload[off], 12);
  off += 12;
  std::memcpy(sample.mag.data(), &event.payload[off], 12);
  off += 12;
  std::memcpy(sample.quat.data(), &event.payload[off], 16);
  off += 16;
  std::memcpy(&sample.temperature, &event.payload[off], 4);
  off += 4;
  sample.calibration_status = event.payload[off];

  imu_queue_->try_push(std::move(sample));
}

void ReplayEngine::dispatchGPS(const RecordEvent &event) {
  if (event.payload.size() < 12 || !gps_callback_)
    return;

  float speed_mps;
  uint64_t orig_ts_ms;
  std::memcpy(&speed_mps, &event.payload[0], 4);
  std::memcpy(&orig_ts_ms, &event.payload[4], 8);

  // Calculate synthetic timestamp (event_offset -> synthetic_ns ->
  // synthetic_ms)
  uint64_t event_offset_ns = event.timestamp_ns - first_event_ts_;
  uint64_t synthetic_ts_ns =
      replay_start_time_ns_ + scaledReplayOffsetNs(event_offset_ns, speed_);
  uint64_t synthetic_ts_ms = synthetic_ts_ns / 1000000;

  gps_callback_(speed_mps, synthetic_ts_ms);
}

} // namespace adas
