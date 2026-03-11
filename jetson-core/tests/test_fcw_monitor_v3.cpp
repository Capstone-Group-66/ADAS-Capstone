// File: tests/test_fcw_monitor_v3.cpp
#include "adas/stage_e/FCWMonitor.hpp"

#include <cassert>
#include <iostream>
#include <vector>

int main() {
  adas::FCWMonitor::Config cfg;
  cfg.camera_hold_ms = 400;
  cfg.caution_dwell_ms = 0;
  cfg.warn_dwell_ms = 0;
  cfg.critical_dwell_ms = 0;
  cfg.clear_dwell_ms = 0;
  cfg.caution_risk_threshold = 0.05f;
  cfg.warn_risk_threshold = 0.10f;
  cfg.critical_risk_threshold = 0.20f;
  cfg.camera_drop_track_hold_ms = 1200;
  cfg.camera_drop_radar_recent_ms = 150;
  cfg.camera_drop_min_quality = 0.32f;

  adas::FCWMonitor monitor(cfg);

  adas::FusedObject obj;
  obj.object_id = 7;
  obj.object_class = 2; // car
  obj.has_radar = true;
  obj.range_m = 6.0f;
  obj.radial_vel_mps = 4.0f;
  obj.ttc_s = 1.5f;
  obj.speed_fresh = true;
  obj.fusion_quality = 0.95f;
  obj.x_lateral_m = 0.1f;
  obj.sources = static_cast<uint16_t>(adas::SRC_CAM_F | adas::SRC_RAD_F);
  obj.camera_age_ms = 500; // stale camera
  obj.radar_age_ms = 300;  // too old for camera-drop grace, should suppress

  std::vector<adas::FusedObject> objects{obj};
  auto alert_stale = monitor.check(objects, 1'000'000'000ULL);
  assert(!alert_stale.has_value());

  objects[0].radar_age_ms = 20; // now eligible for radar-backed camera-drop grace
  auto alert_grace = monitor.check(objects, 1'100'000'000ULL);
  assert(alert_grace.has_value());
  assert(alert_grace->object_id == 7);
  assert(alert_grace->velocity_mps > 0.0f);

  objects[0].camera_age_ms = 200; // now within direct camera hold
  auto alert_fresh = monitor.check(objects, 1'200'000'000ULL);
  assert(alert_fresh.has_value());
  assert(alert_fresh->object_id == 7);

  std::cout << "[PASS] test_fcw_monitor_v3\n";
  return 0;
}
