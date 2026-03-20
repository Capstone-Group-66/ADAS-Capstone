// File: src/stage_a/DeepStreamReceiver.cpp
#include "adas/stage_a/DeepStreamReceiver.hpp"
#include "adas/common/Clock.hpp"
#include "adas/common/DeepStreamIPC.hpp"
#include <chrono>
#include <cstring>
#include <iostream>

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

  while (running_.load()) {
    int len = zmq_recv(ds_socket_, buffer.data(), buffer.size(), 0);
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
    const size_t modern_det_size = sizeof(ipc::DeepStreamDet); // 72 bytes
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
                << (det_record_size == modern_det_size ? " (modern)" :
                    det_record_size == legacy_det_size ? " (legacy)" :
                                                         " (inferred)")
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
          det.cls = ds_det.cls;
          det.score = ds_det.score;
          det.object_id = ds_det.object_id;
        } else {
          LegacyDeepStreamDet ds_det{};
          std::memcpy(&ds_det, buffer.data() + offset, legacy_det_size);
          det.box_px = cv::Rect2f(ds_det.x, ds_det.y, ds_det.w, ds_det.h);
          det.centroid = cv::Point2f(ds_det.centroid_x, ds_det.centroid_y);
          det.cls = ds_det.cls;
          det.score = ds_det.score;
          det.object_id = ds_det.object_id;
        }

        batch.dets.push_back(det);
        offset += det_record_size;

        ids += std::to_string(det.object_id) +
               (i < header.num_detections - 1 ? ", " : "");
      }

      ds_queue_->try_push(std::move(batch));
      // std::cout << "[DeepStream] Received batch: " << header.num_detections
      //           << " detections at " << header.timestamp_ns << " ns. IDs: ["
      //           << ids << "]\n";
    }
  }
}

} // namespace adas
