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

adas::Det makeDetOfClass(uint64_t id, int cls, float cx, float cy, float w,
                         float h) {
  auto d = makeDet(id, cx, cy, w, h);
  d.cls = cls;
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

  // Step 5: range-derived closing should keep speed fresh on a
  // camera-confirmed pedestrian when raw radar speed is stale.
  adas::SensorFusion derived_fusion(cfg);
  const uint64_t td0 = 2'000'000'000ULL;
  adas::DetBatch person_cam;
  person_cam.h.t_device_ns = td0;
  person_cam.h.t_ingest_ns = td0 + 100'000'000ULL;
  person_cam.dets.push_back(makeDetOfClass(
      314, static_cast<int>(adas::ObjectClass::Person), cfg.c_x, 520.0f,
      110.0f, 180.0f));
  derived_fusion.ingestCamera(person_cam);

  auto derived_cam_only = derived_fusion.getFusedObjects(td0 + 20'000'000ULL);
  assert(!derived_cam_only.empty());
  assert(derived_cam_only[0].camera_age_ms == 20);

  float base_range_m = derived_cam_only[0].z_cam_m + cfg.radar_tx_m;
  for (int i = 0; i < 4; ++i) {
    adas::RadarTargets rr;
    rr.h.t_ingest_ns = td0 + static_cast<uint64_t>((i + 1) * 20'000'000ULL);
    adas::RadarTarget rt;
    rt.range_m = base_range_m - 0.10f * static_cast<float>(i);
    rt.radial_vel_mps = 0.0f;
    rt.speed_fresh = false;
    rt.speed_age_ms = 1000;
    rr.targets.push_back(rt);
    derived_fusion.ingestRadar(rr, rr.h.t_ingest_ns);
  }

  auto derived_out = derived_fusion.getFusedObjects(td0 + 80'000'000ULL);
  bool found_derived = false;
  for (const auto &o : derived_out) {
    if (o.object_id == 314) {
      assert(o.has_radar);
      assert(o.speed_fresh);
      assert(o.velocity_source == adas::VelocitySource::DerivedRangeRate);
      assert(o.radial_vel_mps > 1.0f);
      found_derived = true;
    }
  }
  assert(found_derived);

  // Step 6: if raw radar speed disagrees strongly with stable range-derived
  // closing, derived should win.
  adas::SensorFusion disagreement_fusion(cfg);
  const uint64_t td1 = 3'000'000'000ULL;
  adas::DetBatch car_cam;
  car_cam.h.t_ingest_ns = td1;
  car_cam.dets.push_back(makeDet(515, cfg.c_x, 520.0f, 110.0f, 180.0f));
  disagreement_fusion.ingestCamera(car_cam, td1);

  auto disagreement_cam_only = disagreement_fusion.getFusedObjects(td1);
  assert(!disagreement_cam_only.empty());
  base_range_m = disagreement_cam_only[0].z_cam_m + cfg.radar_tx_m;
  for (int i = 0; i < 3; ++i) {
    adas::RadarTargets rr;
    rr.h.t_ingest_ns = td1 + static_cast<uint64_t>((i + 1) * 20'000'000ULL);
    adas::RadarTarget rt;
    rt.range_m = base_range_m - 0.10f * static_cast<float>(i);
    rt.radial_vel_mps = 0.0f;
    rt.speed_fresh = false;
    rt.speed_age_ms = 1000;
    rr.targets.push_back(rt);
    disagreement_fusion.ingestRadar(rr, rr.h.t_ingest_ns);
  }
  adas::RadarTargets disagreement_radar;
  disagreement_radar.h.t_ingest_ns = td1 + 80'000'000ULL;
  adas::RadarTarget disagreement_target;
  disagreement_target.range_m = base_range_m - 0.30f;
  disagreement_target.radial_vel_mps = 20.0f;
  disagreement_target.speed_fresh = true;
  disagreement_target.speed_age_ms = 0;
  disagreement_radar.targets.push_back(disagreement_target);
  disagreement_fusion.ingestRadar(disagreement_radar,
                                  disagreement_radar.h.t_ingest_ns);

  auto disagreement_out =
      disagreement_fusion.getFusedObjects(disagreement_radar.h.t_ingest_ns);
  bool found_disagreement = false;
  for (const auto &o : disagreement_out) {
    if (o.object_id == 515) {
      assert(o.speed_fresh);
      assert(o.velocity_source == adas::VelocitySource::DerivedRangeRate);
      assert(o.radial_vel_mps < 10.0f);
      found_disagreement = true;
    }
  }
  assert(found_disagreement);

  // Step 7: implausible range jumps must invalidate the derived-speed channel.
  adas::SensorFusion jump_fusion(cfg);
  const uint64_t td2 = 4'000'000'000ULL;
  adas::DetBatch jump_cam;
  jump_cam.h.t_ingest_ns = td2;
  jump_cam.dets.push_back(makeDet(616, cfg.c_x, 520.0f, 110.0f, 180.0f));
  jump_fusion.ingestCamera(jump_cam, td2);

  auto jump_cam_only = jump_fusion.getFusedObjects(td2);
  assert(!jump_cam_only.empty());
  base_range_m = jump_cam_only[0].z_cam_m + cfg.radar_tx_m;
  for (int i = 0; i < 4; ++i) {
    adas::RadarTargets rr;
    rr.h.t_ingest_ns = td2 + static_cast<uint64_t>((i + 1) * 20'000'000ULL);
    adas::RadarTarget rt;
    rt.range_m = base_range_m - 0.10f * static_cast<float>(i);
    rt.radial_vel_mps = 0.0f;
    rt.speed_fresh = false;
    rt.speed_age_ms = 1000;
    rr.targets.push_back(rt);
    jump_fusion.ingestRadar(rr, rr.h.t_ingest_ns);
  }
  adas::RadarTargets jump_radar;
  jump_radar.h.t_ingest_ns = td2 + 100'000'000ULL;
  adas::RadarTarget jump_target;
  jump_target.range_m = base_range_m - 1.80f;
  jump_target.radial_vel_mps = 0.0f;
  jump_target.speed_fresh = false;
  jump_target.speed_age_ms = 1000;
  jump_radar.targets.push_back(jump_target);
  jump_fusion.ingestRadar(jump_radar, jump_radar.h.t_ingest_ns);

  auto jump_out = jump_fusion.getFusedObjects(jump_radar.h.t_ingest_ns);
  bool found_jump = false;
  for (const auto &o : jump_out) {
    if (o.object_id == 616) {
      assert(o.has_radar);
      assert(!o.speed_fresh);
      assert(o.velocity_source == adas::VelocitySource::None);
      found_jump = true;
    }
  }
  assert(found_jump);

  // Step 8: road signs should not enter front FCW fusion tracking.
  adas::SensorFusion class_gate_fusion(cfg);
  adas::DetBatch sign_cam;
  sign_cam.h.t_ingest_ns = t0;
  sign_cam.dets.push_back(makeDetOfClass(
      901, static_cast<int>(adas::ObjectClass::RoadSign), cfg.c_x, 520.0f,
      110.0f, 180.0f));
  class_gate_fusion.ingestCamera(sign_cam, sign_cam.h.t_ingest_ns);
  auto sign_out = class_gate_fusion.getFusedObjects(sign_cam.h.t_ingest_ns);
  assert(sign_out.empty());
  std::cout << "[PASS] test_sensor_fusion_async\n";
  return 0;
}
