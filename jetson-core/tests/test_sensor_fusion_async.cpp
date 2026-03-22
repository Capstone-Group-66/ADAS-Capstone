// File: tests/test_sensor_fusion_async.cpp
#include "adas/stage_e/SensorFusion.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

adas::Det makeDet(uint64_t id, float cx, float cy, float w, float h) {
  adas::Det d;
  d.object_id = id;
  d.cls = static_cast<int>(adas::ObjectClass::Car);
  d.score = 0.92f;
  d.box_px = cv::Rect2f(cx - w * 0.5f, cy - h * 0.5f, w, h);
  d.centroid = cv::Point2f(cx, cy);
  return d;
}

} // namespace

int main() {
  adas::FusionConfig cfg;
  cfg.normal_angle_gate_deg = 12.5f;
  cfg.aggressive_angle_gate_deg = 18.0f;
  cfg.ttc_aggressive_s = 3.0f;
  cfg.normal_range_gate_m = 5.5f;
  cfg.aggressive_range_scale = 1.5f;
  cfg.predicted_camera_threshold_ms = 80;

  adas::SensorFusion fusion(cfg);

  const uint64_t t0 = 1'000'000'000ULL;

  // Step 1: camera-only track creation.
  adas::DetBatch cam0;
  cam0.h.t_ingest_ns = t0;
  cam0.dets.push_back(makeDet(42, cfg.c_x, 520.0f, 110.0f, 180.0f));
  fusion.ingestCamera(cam0, t0);

  auto f0 = fusion.getFusedObjects(t0);
  assert(!f0.empty());
  assert(f0[0].object_id == 42);
  assert(!f0[0].has_radar);

  // Step 2: radar update should attach and provide authoritative longitudinal state.
  adas::RadarTargets r0;
  r0.h.t_ingest_ns = t0 + 20'000'000ULL;
  adas::RadarTarget rt0;
  rt0.range_m = f0[0].z_cam_m + cfg.radar_tx_m;
  rt0.radial_vel_mps = 3.0f;
  rt0.speed_fresh = true;
  rt0.speed_age_ms = 0;
  r0.targets.push_back(rt0);
  fusion.ingestRadar(r0, r0.h.t_ingest_ns);

  auto f1 = fusion.getFusedObjects(r0.h.t_ingest_ns);
  assert(!f1.empty());
  assert(f1[0].has_radar);
  assert(f1[0].speed_fresh);
  assert(f1[0].range_m > 0.1f);

  // Step 3: no new camera frame -> track should become predicted camera.
  auto f2 = fusion.getFusedObjects(r0.h.t_ingest_ns + 180'000'000ULL);
  assert(!f2.empty());
  assert(f2[0].is_predicted_camera);

  // Step 4: aggressive association check.
  // Prime track with in-gate theta then move camera estimate outside normal gate but
  // inside aggressive gate while TTC remains < 3s.
  const uint64_t t1 = t0 + 400'000'000ULL;
  adas::DetBatch cam1;
  cam1.h.t_ingest_ns = t1;
  const float theta10 = 10.0f * 3.14159265f / 180.0f;
  const float cx10 = cfg.c_x + cfg.f_x * std::tan(theta10);
  cam1.dets.push_back(makeDet(77, cx10, 525.0f, 108.0f, 175.0f));
  fusion.ingestCamera(cam1, t1);

  adas::RadarTargets r1;
  r1.h.t_ingest_ns = t1 + 10'000'000ULL;
  adas::RadarTarget rt1;
  rt1.range_m = 6.0f + cfg.radar_tx_m;
  rt1.radial_vel_mps = 3.4f; // TTC ~1.76s
  rt1.speed_fresh = true;
  rt1.speed_age_ms = 0;
  r1.targets.push_back(rt1);
  fusion.ingestRadar(r1, r1.h.t_ingest_ns);

  adas::DetBatch cam2;
  cam2.h.t_ingest_ns = t1 + 25'000'000ULL;
  const float theta16 = 16.0f * 3.14159265f / 180.0f;
  const float cx16 = cfg.c_x + cfg.f_x * std::tan(theta16);
  cam2.dets.push_back(makeDet(77, cx16, 525.0f, 108.0f, 175.0f));
  fusion.ingestCamera(cam2, cam2.h.t_ingest_ns);

  adas::RadarTargets r2;
  r2.h.t_ingest_ns = t1 + 40'000'000ULL;
  adas::RadarTarget rt2;
  rt2.range_m = 5.8f + cfg.radar_tx_m;
  rt2.radial_vel_mps = 3.3f;
  rt2.speed_fresh = true;
  rt2.speed_age_ms = 0;
  r2.targets.push_back(rt2);
  fusion.ingestRadar(r2, r2.h.t_ingest_ns);

  auto f3 = fusion.getFusedObjects(r2.h.t_ingest_ns);
  bool found_aggressive = false;
  for (const auto &o : f3) {
    if (o.object_id == 77) {
      assert(o.has_radar);
      found_aggressive = o.is_aggressive_mode;
    }
  }
  assert(found_aggressive);

  std::cout << "[PASS] test_sensor_fusion_async\n";
  return 0;
}
