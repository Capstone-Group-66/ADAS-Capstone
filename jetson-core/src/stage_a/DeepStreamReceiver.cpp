// File: src/stage_a/DeepStreamReceiver.cpp
#include "adas/stage_a/DeepStreamReceiver.hpp"
#include "adas/common/Clock.hpp"
#include "adas/common/DeepStreamIPC.hpp"
#include <chrono>
#include <cstring>
#include <iostream>

namespace adas {

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

    size_t expected_size = sizeof(ipc::DeepStreamDetBatchHeader) +
                           header.num_detections * sizeof(ipc::DeepStreamDet);

    if (static_cast<size_t>(len) < expected_size) {
      std::cerr << "[DeepStream] Dropped packet: incomplete batch (" << len
                << " bytes, expected " << expected_size << ")\n";
      continue;
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
      for (uint32_t i = 0; i < header.num_detections; ++i) {
        ipc::DeepStreamDet ds_det;
        std::memcpy(&ds_det, buffer.data() + offset,
                    sizeof(ipc::DeepStreamDet));

        Det det;
        det.box_px = cv::Rect2f(ds_det.x, ds_det.y, ds_det.w, ds_det.h);
        det.centroid = cv::Point2f(ds_det.centroid_x, ds_det.centroid_y);
        det.cls = ds_det.cls;
        det.score = ds_det.score;
        det.object_id = ds_det.object_id;

        batch.dets.push_back(det);
        offset += sizeof(ipc::DeepStreamDet);
      }

      ds_queue_->try_push(std::move(batch));
      std::cout << "[DeepStream] Received batch: " << header.num_detections 
                << " detections at " << header.timestamp_ns << " ns\n";
    }
  }
}

} // namespace adas
