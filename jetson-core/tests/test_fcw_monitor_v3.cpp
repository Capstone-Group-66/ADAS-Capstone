// File: tests/test_fcw_monitor_v3.cpp
#include "adas/stage_e/FCWMonitor.hpp"

#include <cassert>
#include <iostream>
#include <vector>

int main() {
  // Existing camera-drop grace behavior regression checks.
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

  // New behavior: immediate TTC escalation should bypass dwell.
  adas::FCWMonitor::Config immediate_cfg;
  immediate_cfg.caution_dwell_ms = 180;
  immediate_cfg.warn_dwell_ms = 140;
  immediate_cfg.critical_dwell_ms = 80;
  immediate_cfg.clear_dwell_ms = 220;
  immediate_cfg.ttc_threshold_s = 3.0f;
  immediate_cfg.ttc_immediate_warn_s = 2.8f;
  immediate_cfg.ttc_immediate_critical_s = 1.2f;
  immediate_cfg.min_fusion_quality = 0.2f;
  immediate_cfg.log_fcw_drop_reasons = false;
  adas::FCWMonitor immediate_monitor(immediate_cfg);

  adas::FusedObject ttc_obj;
  ttc_obj.object_id = 99;
  ttc_obj.object_class = 2;
  ttc_obj.has_radar = true;
  ttc_obj.range_m = 3.2f;
  ttc_obj.radial_vel_mps = 1.4f;
  ttc_obj.ttc_s = 2.2f;
  ttc_obj.speed_fresh = true;
  ttc_obj.fusion_quality = 0.25f;
  ttc_obj.x_lateral_m = 0.0f;
  ttc_obj.sources = static_cast<uint16_t>(adas::SRC_CAM_F | adas::SRC_RAD_F);
  ttc_obj.camera_age_ms = 50;
  ttc_obj.radar_age_ms = 20;

  std::vector<adas::FusedObject> ttc_objects{ttc_obj};
  auto immediate_warn = immediate_monitor.check(ttc_objects, 2'000'000'000ULL);
  assert(immediate_warn.has_value());
  const auto eval_warn = immediate_monitor.getLastEvaluation();
  assert(eval_warn.has_candidate);
  assert(eval_warn.level >= static_cast<uint8_t>(adas::FCWMonitor::RiskLevel::Warn));

  ttc_objects[0].ttc_s = 1.0f;
  auto immediate_critical =
      immediate_monitor.check(ttc_objects, 2'100'000'000ULL);
  assert(immediate_critical.has_value());
  const auto eval_critical = immediate_monitor.getLastEvaluation();
  assert(eval_critical.has_candidate);
  assert(eval_critical.level ==
         static_cast<uint8_t>(adas::FCWMonitor::RiskLevel::Critical));

  ttc_objects[0].x_lateral_m = 8.0f; // Out-of-path should suppress escalation.
  auto out_of_path =
      immediate_monitor.check(ttc_objects, 2'200'000'000ULL);
  assert(!out_of_path.has_value());

  // New behavior: brief invalid bursts should not instantly wipe active state.
  adas::FCWMonitor::Config hold_cfg = immediate_cfg;
  hold_cfg.clear_dwell_ms = 0;
  hold_cfg.invalid_state_hold_ms = 250;
  adas::FCWMonitor hold_monitor(hold_cfg);

  adas::FusedObject hold_obj = ttc_obj;
  hold_obj.object_id = 123;
  hold_obj.x_lateral_m = 0.1f;
  hold_obj.ttc_s = 2.0f;
  hold_obj.speed_fresh = true;
  std::vector<adas::FusedObject> hold_objects{hold_obj};
  auto hold_warn = hold_monitor.check(hold_objects, 3'000'000'000ULL);
  assert(hold_warn.has_value());

  hold_objects[0].speed_fresh = false;
  auto hold_gap_1 = hold_monitor.check(hold_objects, 3'100'000'000ULL);
  assert(!hold_gap_1.has_value());
  auto hold_gap_2 = hold_monitor.check(hold_objects, 3'200'000'000ULL);
  assert(!hold_gap_2.has_value());

  hold_objects[0].speed_fresh = true;
  hold_objects[0].ttc_s = 3.5f; // No immediate TTC escalation on recovery tick.
  auto hold_recovery = hold_monitor.check(hold_objects, 3'210'000'000ULL);
  assert(hold_recovery.has_value()); // Active WARN preserved through short gap.

  auto hold_demote = hold_monitor.check(hold_objects, 3'600'000'000ULL);
  assert(!hold_demote.has_value()); // Hold window exceeded, demoted to safe.

  std::cout << "[PASS] test_fcw_monitor_v3\n";
  return 0;
}
