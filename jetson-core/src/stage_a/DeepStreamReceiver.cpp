// File: src/stage_a/DeepStreamReceiver.cpp
#include "adas/stage_a/DeepStreamReceiver.hpp"
#include "adas/common/Clock.hpp"
#include "adas/common/DeepStreamIPC.hpp"
#include "adas/recording/Recorder.hpp"
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_map>

namespace adas {

namespace {

#pragma pack(push, 1)
struct LegacyDeepStreamDet {
  float x;
  float y;
  float w;
  float h;
  float centroid_x;
  float centroid_y;
  int32_t cls;
  float score;
  uint64_t object_id;
};
#pragma pack(pop)
static_assert(sizeof(LegacyDeepStreamDet) == 40,
              "LegacyDeepStreamDet must be 40 bytes");

constexpr uint64_t kRoadSignLogWindowNs = 2000000000ULL;

std::string roadSignLabel(const Det &det) {
  if (det.hasSignLabel()) {
    return det.signLabelString();
  }
  return "RoadSign";
}

struct RoadSignSummary {
  int count = 0;
  float best_score = -1.0f;
  Det best_det{};
};

class RoadSignLogAccumulator {
public:
  using SummaryMap = std::unordered_map<std::string, RoadSignSummary>;
  using SummaryConstIterator = SummaryMap::const_iterator;

  void observe(const Det &det, uint64_t now_ns) {
    if (!active_) {
      active_ = true;
      window_start_ns_ = now_ns;
    }

    auto &summary = summaries_[roadSignLabel(det)];
    summary.count += 1;
    if (det.score >= summary.best_score) {
      summary.best_score = det.score;
      summary.best_det = det;
    }
  }

  void flushIfDue(uint64_t now_ns, bool force = false) {
    if (!active_) {
      return;
    }
    if (!force && now_ns < window_start_ns_ + kRoadSignLogWindowNs) {
      return;
    }

    const auto best_it = chooseDominantSummary();
    if (best_it != summaries_.end()) {
      const auto &label = best_it->first;
      const auto &summary = best_it->second;
      const Det &det = summary.best_det;

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2)
         << "[RoadSignDet] 2s summary | sign: " << label
         << " | hits: " << summary.count << " | best score: " << det.score
         << " | box: [" << det.box_px.x << ", " << det.box_px.y << ", "
         << det.box_px.width << ", " << det.box_px.height << "]";
      std::cout << ss.str() << "\n";
    }

    summaries_.clear();
    active_ = false;
    window_start_ns_ = 0;
  }

private:
  SummaryConstIterator chooseDominantSummary() const {
    auto best_it = summaries_.end();
    for (auto it = summaries_.begin(); it != summaries_.end(); ++it) {
      if (best_it == summaries_.end() ||
          it->second.count > best_it->second.count ||
          (it->second.count == best_it->second.count &&
           it->second.best_score > best_it->second.best_score)) {
        best_it = it;
      }
    }
    return best_it;
  }

  bool active_ = false;
  uint64_t window_start_ns_ = 0;
  SummaryMap summaries_;
};

RoadSignLogAccumulator makeRoadSignLogAccumulator() {
  return RoadSignLogAccumulator();
}

} // namespace

DeepStreamReceiver::DeepStreamReceiver() { context_ = zmq_ctx_new(); }

DeepStreamReceiver::~DeepStreamReceiver() {
  stop();
  if (context_) {
    zmq_ctx_term(context_);
    context_ = nullptr;
  }
}

bool DeepStreamReceiver::start(SPSCQueue<DetBatch, 8> *ds_queue) {
  if (running_.load()) {
    return true;
  }

  ds_queue_ = ds_queue;

  ds_socket_ = zmq_socket(context_, ZMQ_PULL);

  int hwm = 10;
  zmq_setsockopt(ds_socket_, ZMQ_RCVHWM, &hwm, sizeof(hwm));

  int timeout = 1000;
  zmq_setsockopt(ds_socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

  if (zmq_bind(ds_socket_, "ipc:///tmp/ds_front_cam.sock") != 0) {
    std::cerr << "[DeepStreamReceiver] Failed to bind IPC socket\n";
    return false;
  }

  std::cout << "[DeepStreamReceiver] Bound local IPC socket for front camera\n";

  running_.store(true);
  ds_thread_ = std::thread(&DeepStreamReceiver::dsThread, this);

  return true;
}

void DeepStreamReceiver::stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  if (ds_socket_) {
    // Force context term to unblock recv if we want, or just wait for timeout.
    // We set RCVTIMEO so thread will exit quickly.
  }

  if (ds_thread_.joinable()) {
    ds_thread_.join();
  }

  if (ds_socket_) {
    zmq_close(ds_socket_);
    ds_socket_ = nullptr;
  }

  std::cout << "[DeepStreamReceiver] Stopped\n";
}

void DeepStreamReceiver::dsThread() {
  std::vector<uint8_t> buffer(1024 * 1024); // 1MB buffer
  auto road_sign_logger = makeRoadSignLogAccumulator();

  while (running_.load()) {
    int len = zmq_recv(ds_socket_, buffer.data(), buffer.size(), 0);
    const uint64_t now_ns = Clock::now_ns();
    road_sign_logger.flushIfDue(now_ns);
    if (len < 0) {
      continue; // Timeout or error
    }

    if (static_cast<size_t>(len) < sizeof(ipc::DeepStreamDetBatchHeader)) {
      continue;
    }

    ipc::DeepStreamDetBatchHeader header;
    std::memcpy(&header, buffer.data(), sizeof(header));

    const size_t payload_size =
        static_cast<size_t>(len) - sizeof(ipc::DeepStreamDetBatchHeader);
    const size_t modern_det_size = sizeof(ipc::DeepStreamDet);  // 72 bytes
    const size_t legacy_det_size = sizeof(LegacyDeepStreamDet); // 40 bytes
    size_t det_record_size = 0;

    if (header.num_detections == 0) {
      continue;
    }

    if (payload_size == header.num_detections * modern_det_size) {
      det_record_size = modern_det_size;
    } else if (payload_size == header.num_detections * legacy_det_size) {
      det_record_size = legacy_det_size;
    } else if (payload_size % header.num_detections == 0) {
      const size_t inferred_size = payload_size / header.num_detections;
      if (inferred_size >= legacy_det_size) {
        det_record_size = inferred_size;
      }
    }

    if (det_record_size == 0) {
      std::cerr << "[DeepStream] Dropped packet: layout mismatch (total=" << len
                << " payload=" << payload_size
                << " num_detections=" << header.num_detections
                << " modern_det_size=" << modern_det_size
                << " legacy_det_size=" << legacy_det_size << ")\n";
      continue;
    }

    static size_t last_logged_det_size = 0;
    if (det_record_size != last_logged_det_size) {
      std::cout << "[DeepStreamReceiver] Det record layout detected: "
                << det_record_size << " bytes"
                << (det_record_size == modern_det_size   ? " (modern)"
                    : det_record_size == legacy_det_size ? " (legacy)"
                                                         : " (inferred)")
                << "\n";
      last_logged_det_size = det_record_size;
    }

    if (ds_queue_) {
      DetBatch batch;
      batch.h.mount = Mount::FrontCam;
      batch.h.t_device_ns =
          header.timestamp_ns; // Provided by DeepStream (CLOCK_MONOTONIC)
      batch.h.t_ingest_ns = Clock::now_ns(); // Arrival time on this thread
      batch.h.seq = 0; // Not explicitly tracked via ZMQ header here currently
      batch.h.healthy = true;
      batch.inference_time_us = 0; // Info not provided currently

      size_t offset = sizeof(ipc::DeepStreamDetBatchHeader);
      std::string ids = "";
      for (uint32_t i = 0; i < header.num_detections; ++i) {
        Det det;
        if (det_record_size >= modern_det_size) {
          ipc::DeepStreamDet ds_det{};
          std::memcpy(&ds_det, buffer.data() + offset, modern_det_size);
          det.box_px = cv::Rect2f(ds_det.x, ds_det.y, ds_det.w, ds_det.h);
          det.centroid = cv::Point2f(ds_det.centroid_x, ds_det.centroid_y);
          det.cls = static_cast<int>(deepStreamToObjectClass(ds_det.cls));
          det.score = ds_det.score;
          det.object_id = ds_det.object_id;
          det.setSignLabel(ds_det.sign_type);

          if (det.cls == 3 || det.hasSignLabel()) {
            road_sign_logger.observe(det, now_ns);
          }
        } else {
          LegacyDeepStreamDet ds_det{};
          std::memcpy(&ds_det, buffer.data() + offset, legacy_det_size);
          det.box_px = cv::Rect2f(ds_det.x, ds_det.y, ds_det.w, ds_det.h);
          det.centroid = cv::Point2f(ds_det.centroid_x, ds_det.centroid_y);
          det.cls = static_cast<int>(deepStreamToObjectClass(ds_det.cls));
          det.score = ds_det.score;
          det.object_id = ds_det.object_id;
        }

        batch.dets.push_back(det);
        offset += det_record_size;

        ids += std::to_string(det.object_id) +
               (i < header.num_detections - 1 ? ", " : "");
      }

      if (auto *rec = recorder_.load(std::memory_order_acquire)) {
        rec->recordFrontDetections(batch);
      }

      ds_queue_->try_push(std::move(batch));
      // std::cout << "[DeepStream] Received batch: " << header.num_detections
      //           << " detections at " << header.timestamp_ns << " ns. IDs: ["
      //           << ids << "]\n";
    }
  }

  road_sign_logger.flushIfDue(Clock::now_ns(), true);
}

} // namespace adas
