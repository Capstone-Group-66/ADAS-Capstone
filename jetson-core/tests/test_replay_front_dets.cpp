// File: tests/test_replay_front_dets.cpp
#include "adas/queues/SPSCQueue.hpp"
#include "adas/recording/Recorder.hpp"
#include "adas/recording/ReplayEngine.hpp"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

using namespace adas;

namespace {

bool waitForReplay(ReplayEngine &replay, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!replay.isFinished() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return replay.isFinished();
}

bool testFrontDetRoundTrip(const std::filesystem::path &dir) {
  Recorder recorder;
  if (!recorder.start(dir.string())) {
    std::cerr << "[FAIL] Could not start recorder\n";
    return false;
  }

  DetBatch batch;
  batch.h.mount = Mount::FrontCam;
  batch.h.t_ingest_ns = 1'000'000'000ULL;
  batch.h.t_device_ns = 999'500'000ULL;
  batch.h.seq = 7;
  batch.h.healthy = true;
  batch.inference_time_us = 12345;
  batch.dets.emplace_back(cv::Rect2f(10.0f, 20.0f, 30.0f, 40.0f),
                          static_cast<int>(ObjectClass::RoadSign), 0.91f, 55,
                          "Stop");
  batch.dets.emplace_back(cv::Rect2f(50.0f, 60.0f, 70.0f, 80.0f),
                          static_cast<int>(ObjectClass::Car), 0.87f, 77,
                          nullptr);

  recorder.recordFrontDetections(batch);

  DetBatch empty_batch;
  empty_batch.h.mount = Mount::FrontCam;
  empty_batch.h.t_ingest_ns = 1'100'000'000ULL;
  empty_batch.h.t_device_ns = 1'099'000'000ULL;
  empty_batch.h.seq = 8;
  empty_batch.h.healthy = true;
  empty_batch.inference_time_us = 777;
  recorder.recordFrontDetections(empty_batch);
  recorder.stop();

  std::ifstream in(recorder.getFilePath(), std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "[FAIL] Could not reopen v2 recording\n";
    return false;
  }
  AdasRecFileHeader header;
  in.read(reinterpret_cast<char *>(&header), sizeof(header));
  if (header.version != 2) {
    std::cerr << "[FAIL] Expected v2 recording header\n";
    return false;
  }

  ReplayEngine replay;
  if (!replay.load(recorder.getFilePath())) {
    std::cerr << "[FAIL] Could not load v2 replay file\n";
    return false;
  }

  SPSCQueue<DetBatch, 8> front_det_queue;
  replay.setFrontDetQueue(&front_det_queue);
  replay.setSpeed(1.0f);
  replay.start();
  if (!waitForReplay(replay, std::chrono::milliseconds(500))) {
    std::cerr << "[FAIL] Replay did not finish for v2 file\n";
    replay.stop();
    return false;
  }
  replay.stop();

  DetBatch replayed;
  DetBatch replayed_empty;
  if (!front_det_queue.try_pop(replayed) ||
      !front_det_queue.try_pop(replayed_empty)) {
    std::cerr << "[FAIL] Expected two replayed front detection batches from v2 "
                 "file\n";
    return false;
  }

  bool ok = replayed.h.mount == Mount::FrontCam &&
            replayed.inference_time_us == batch.inference_time_us &&
            replayed.dets.size() == batch.dets.size() &&
            replayed.dets[0].object_id == 55 &&
            replayed.dets[0].cls == static_cast<int>(ObjectClass::RoadSign) &&
            replayed.dets[0].signLabelString() == "Stop" &&
            replayed.dets[1].object_id == 77 &&
            replayed.dets[1].cls == static_cast<int>(ObjectClass::Car) &&
            !replayed.dets[1].hasSignLabel() && replayed.h.t_ingest_ns > 0 &&
            replayed.h.t_device_ns == replayed.h.t_ingest_ns &&
            replayed_empty.h.mount == Mount::FrontCam &&
            replayed_empty.dets.empty() &&
            replayed_empty.inference_time_us == empty_batch.inference_time_us &&
            replayed_empty.h.t_ingest_ns > replayed.h.t_ingest_ns &&
            replayed_empty.h.t_device_ns == replayed_empty.h.t_ingest_ns;

  if (!ok) {
    std::cerr << "[FAIL] Replayed front detection batch did not match input\n";
    return false;
  }

  std::cout << "[PASS] v2 front detection round-trip\n";
  return true;
}

bool writeLegacyV1File(const std::filesystem::path &path) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    return false;
  }

  AdasRecFileHeader header;
  header.version = 1;
  header.start_time_ns = 2'000'000'000ULL;
  out.write(reinterpret_cast<const char *>(&header), sizeof(header));

  RecEventHeader event_header;
  event_header.timestamp_ns = 2'000'000'000ULL;
  event_header.type = static_cast<uint8_t>(RecEventType::GPS);
  event_header.payload_size = 32;
  out.write(reinterpret_cast<const char *>(&event_header), sizeof(event_header));

  std::vector<uint8_t> payload(32, 0);
  float speed_mps = 13.5f;
  uint64_t ts_ms = 4242;
  std::memcpy(payload.data(), &speed_mps, sizeof(speed_mps));
  std::memcpy(payload.data() + 4, &ts_ms, sizeof(ts_ms));
  out.write(reinterpret_cast<const char *>(payload.data()), payload.size());
  return true;
}

bool testLegacyV1Load(const std::filesystem::path &dir) {
  const auto path = dir / "legacy_v1.adasrec";
  if (!writeLegacyV1File(path)) {
    std::cerr << "[FAIL] Could not write legacy v1 file\n";
    return false;
  }

  ReplayEngine replay;
  if (!replay.load(path.string())) {
    std::cerr << "[FAIL] Could not load legacy v1 file\n";
    return false;
  }

  SPSCQueue<DetBatch, 8> front_det_queue;
  replay.setFrontDetQueue(&front_det_queue);

  float last_speed = 0.0f;
  uint64_t last_ts_ms = 0;
  replay.setGpsCallback([&](float speed_mps, uint64_t ts_ms) {
    last_speed = speed_mps;
    last_ts_ms = ts_ms;
  });

  replay.setSpeed(1.0f);
  replay.start();
  if (!waitForReplay(replay, std::chrono::milliseconds(500))) {
    std::cerr << "[FAIL] Replay did not finish for legacy v1 file\n";
    replay.stop();
    return false;
  }
  replay.stop();

  DetBatch unexpected;
  const bool ok =
      last_speed == 13.5f && last_ts_ms > 0 && !front_det_queue.try_pop(unexpected);
  if (!ok) {
    std::cerr << "[FAIL] Legacy v1 replay behavior was incorrect\n";
    return false;
  }

  std::cout << "[PASS] legacy v1 load without front detections\n";
  return true;
}

} // namespace

int main() {
  const auto dir =
      std::filesystem::current_path() / "test_artifacts_replay_front_dets";
  std::filesystem::create_directories(dir);

  int failed = 0;
  failed += testFrontDetRoundTrip(dir) ? 0 : 1;
  failed += testLegacyV1Load(dir) ? 0 : 1;

  if (failed > 0) {
    std::cerr << "[FAIL] test_replay_front_dets: " << failed
              << " scenario(s) failed\n";
    return 1;
  }

  std::cout << "[PASS] test_replay_front_dets complete\n";
  return 0;
}
