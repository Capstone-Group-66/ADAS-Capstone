// File: tests/test_sensor_fusion_async.cpp
#include "adas/stage_e/SensorFusion.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

adas::Det makeForwardDet(uint64_t id, const adas::FusionConfig &cfg,
                         float range_m, float theta_deg = 0.0f,
                         float w = 110.0f, float h = 180.0f) {
  adas::Det d;
  d.object_id = id;
  d.cls = static_cast<int>(adas::ObjectClass::Car);
  d.score = 0.92f;

  const float theta_rad = theta_deg * 3.14159265f / 180.0f;
  const float cx = cfg.c_x + cfg.f_x * std::tan(theta_rad);
  const float v_bottom = cfg.c_y + (cfg.f_y * cfg.cam_height_m / range_m);
  const float cy = v_bottom - h * 0.5f;

  d.box_px = cv::Rect2f(cx - w * 0.5f, cy - h * 0.5f, w, h);
  d.centroid = cv::Point2f(cx, cy);
  return d;
}

adas::RadarTargets makeRadarTick(uint64_t t_ns, float range_m,
                                 float range_tx_m, bool speed_fresh,
                                 float radial_vel_mps = 0.0f,
                                 uint32_t speed_age_ms = 0) {
  adas::RadarTargets radar;
  radar.h.t_ingest_ns = t_ns;
  adas::RadarTarget target;
  target.range_m = range_m + range_tx_m;
  target.radial_vel_mps = radial_vel_mps;
  target.speed_fresh = speed_fresh;
  target.speed_age_ms = speed_age_ms;
  radar.targets.push_back(target);
  return radar;
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
  cfg.derived_speed_min_hits = 3;
  cfg.derived_speed_min_dt_ms = 20;
  cfg.derived_speed_max_dt_ms = 250;
  cfg.derived_speed_hold_ms = 200;
  cfg.provisional_min_hits = 3;
  cfg.promotion_min_hits = 3;

  const uint64_t t0 = 1'000'000'000ULL;

  // Step 1: camera-only track creation.
  adas::SensorFusion fusion(cfg);
  adas::DetBatch cam0;
  cam0.h.t_ingest_ns = t0;
  cam0.dets.push_back(makeForwardDet(42, cfg, 12.0f));
  fusion.ingestCamera(cam0, t0);

  auto f0 = fusion.getFusedObjects(t0);
  assert(!f0.empty());
  assert(f0[0].object_id == 42);
  assert(!f0[0].has_radar);

  // Step 2: radar speed update should attach and provide authoritative state.
  auto r0 = makeRadarTick(t0 + 20'000'000ULL, f0[0].z_cam_m, cfg.radar_tx_m,
                          true, 3.0f, 0);
  fusion.ingestRadar(r0, r0.h.t_ingest_ns);

  auto f1 = fusion.getFusedObjects(r0.h.t_ingest_ns);
  assert(!f1.empty());
  assert(f1[0].has_radar);
  assert(f1[0].speed_fresh);
  assert(f1[0].range_m > 0.1f);
  assert(f1[0].radial_velocity_source ==
         adas::RadialVelocitySource::RadarSensor);

  // Step 3: no new camera frame -> track should become predicted camera.
  auto f2 = fusion.getFusedObjects(r0.h.t_ingest_ns + 180'000'000ULL);
  assert(!f2.empty());
  assert(f2[0].is_predicted_camera);

  // Step 4: aggressive association check still works.
  const uint64_t t1 = t0 + 400'000'000ULL;
  adas::DetBatch cam1;
  cam1.h.t_ingest_ns = t1;
  cam1.dets.push_back(makeForwardDet(77, cfg, 6.1f, 10.0f, 108.0f, 175.0f));
  fusion.ingestCamera(cam1, t1);

  auto r1 = makeRadarTick(t1 + 10'000'000ULL, 6.0f, cfg.radar_tx_m, true, 3.4f,
                          0);
  fusion.ingestRadar(r1, r1.h.t_ingest_ns);

  adas::DetBatch cam2;
  cam2.h.t_ingest_ns = t1 + 25'000'000ULL;
  cam2.dets.push_back(makeForwardDet(77, cfg, 5.9f, 16.0f, 108.0f, 175.0f));
  fusion.ingestCamera(cam2, cam2.h.t_ingest_ns);

  auto r2 = makeRadarTick(t1 + 40'000'000ULL, 5.8f, cfg.radar_tx_m, true, 3.3f,
                          0);
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

  // Step 5: range-only radar hits should derive closing speed for a confirmed track.
  adas::SensorFusion derived_fusion(cfg);
  adas::DetBatch cam_range;
  cam_range.h.t_ingest_ns = t0;
  cam_range.dets.push_back(makeForwardDet(100, cfg, 18.0f));
  derived_fusion.ingestCamera(cam_range, t0);

  auto rr0 = makeRadarTick(t0 + 20'000'000ULL, 18.0f, cfg.radar_tx_m, false);
  auto rr1 = makeRadarTick(t0 + 70'000'000ULL, 17.6f, cfg.radar_tx_m, false);
  auto rr2 = makeRadarTick(t0 + 120'000'000ULL, 17.2f, cfg.radar_tx_m, false);
  derived_fusion.ingestRadar(rr0, rr0.h.t_ingest_ns);
  derived_fusion.ingestRadar(rr1, rr1.h.t_ingest_ns);
  derived_fusion.ingestRadar(rr2, rr2.h.t_ingest_ns);

  auto fd = derived_fusion.getFusedObjects(rr2.h.t_ingest_ns);
  assert(!fd.empty());
  assert(fd[0].has_radar);
  assert(fd[0].speed_fresh);
  assert(fd[0].radial_vel_mps > 0.1f);
  assert(fd[0].radial_velocity_source ==
         adas::RadialVelocitySource::DerivedRangeRate);

  // Step 6: radar-only provisional track should promote on first strong camera match.
  adas::SensorFusion provisional_fusion(cfg);
  auto pr0 = makeRadarTick(t0 + 10'000'000ULL, 20.0f, cfg.radar_tx_m, false);
  auto pr1 = makeRadarTick(t0 + 60'000'000ULL, 19.6f, cfg.radar_tx_m, false);
  auto pr2 = makeRadarTick(t0 + 110'000'000ULL, 19.2f, cfg.radar_tx_m, false);
  provisional_fusion.ingestRadar(pr0, pr0.h.t_ingest_ns);
  provisional_fusion.ingestRadar(pr1, pr1.h.t_ingest_ns);
  provisional_fusion.ingestRadar(pr2, pr2.h.t_ingest_ns);
  auto provisional_only = provisional_fusion.getFusedObjects(pr2.h.t_ingest_ns);
  assert(provisional_only.empty());

  adas::DetBatch late_cam;
  late_cam.h.t_ingest_ns = t0 + 120'000'000ULL;
  late_cam.dets.push_back(makeForwardDet(555, cfg, 19.2f));
  provisional_fusion.ingestCamera(late_cam, late_cam.h.t_ingest_ns);
  auto promoted = provisional_fusion.getFusedObjects(late_cam.h.t_ingest_ns);
  assert(!promoted.empty());
  assert(promoted[0].object_id == 555);
  assert(promoted[0].has_radar);
  assert(promoted[0].speed_fresh);
  assert(promoted[0].radial_vel_mps > 0.1f);
  assert(promoted[0].radial_velocity_source ==
         adas::RadialVelocitySource::DerivedRangeRate);

  // Step 7: camera freshness should prefer device time when provided.
  adas::SensorFusion freshness_fusion(cfg);
  adas::DetBatch delayed_cam;
  delayed_cam.h.t_device_ns = t0;
  delayed_cam.h.t_ingest_ns = t0 + 500'000'000ULL;
  delayed_cam.dets.push_back(makeForwardDet(900, cfg, 15.0f));
  freshness_fusion.ingestCamera(delayed_cam, delayed_cam.h.t_ingest_ns);
  auto delayed_out =
      freshness_fusion.getFusedObjects(delayed_cam.h.t_ingest_ns);
  assert(!delayed_out.empty());
  assert(delayed_out[0].camera_age_ms >= 450);

  std::cout << "[PASS] test_sensor_fusion_async\n";
  return 0;
}
