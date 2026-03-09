// File: src/stage_e/SensorFusion.cpp
// Ground-plane 1D–2D Sensor Fusion implementation.
//
// Mathematics (per user spec):
//
// Phase 2 — Camera Distance Estimation
//   u  = box bottom-centre x  (u always used for ROI horizontal check)
//   v  = box.y + box.height   (bottom-centre pixel row)
//   α  = atan2(v - c_y, f_y)  (vertical optical angle below horizon)
//   Z_cam = H / tan(θ + α)    (ground-plane range, θ=pitch from IMU)
//
// Phase 3 — Camera Velocity Estimation
//   V_cam = (Z_cam(t) - Z_cam(t-Δt)) / Δt      (m/s)
//   Negative V_cam = approaching (Z decreasing)
//
// Phase 4 — Radar: Z_rad = target.range_m
//           V_rad = target.radial_vel_mps  (OPS243-A: positive = approaching)
//
// Phase 5 — Gating (all 3 must pass):
//   1. (u, v)  inside radar ROI bounding box
//   2. |Z_cam - Z_rad|  < dist_gate_m
//   3. |V_cam - V_rad|  < vel_gate_mps    (both normalised to
//   negative=approaching)
//
// TTC = Z_rad / |V_rad|   (radar range is authoritative)
#include "adas/stage_e/SensorFusion.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "adas/common/Globals.hpp"

namespace adas {

SensorFusion::SensorFusion(const FusionConfig &config) : config_(config) {}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────

cv::Rect2f SensorFusion::computeRadarROI(float frame_width,
                                         float frame_height) const {
  // Scale intrinsics from calibration resolution to current resolution.
  // This makes the ROI robust to resolution changes (e.g. 1280×720 → 1024×540).
  const float sx = frame_width / config_.calib_width_px;
  const float sy = frame_height / config_.calib_height_px;

  const float fx_s = config_.f_x * sx;
  const float fy_s = config_.f_y * sy;
  const float cx_s = config_.c_x * sx;
  const float cy_s = config_.c_y * sy;

  const float tan_half =
      std::tan(config_.radar_half_fov_deg * (float)M_PI / 180.f);

  // Horizontal extent: ±half_fov from camera principal point.
  // Camera and radar share the same horizontal boresight axis.
  const float roi_x_min = cx_s - fx_s * tan_half;
  const float roi_x_max = cx_s + fx_s * tan_half;

  // Vertical centre: the radar is physically radar_below_cam_m below the
  // camera.  At reference depth roi_z_ref_m, this translates to a downward
  // pixel offset of:  Δv = fy_s * radar_below_cam_m / roi_z_ref_m
  // For roi_z_ref_m=20m: Δv ≈ 829*0.0762/20 ≈ 3.2 px — negligible but correct.
  const float v_center =
      cy_s + fy_s * (config_.radar_below_cam_m / config_.roi_z_ref_m);

  // Vertical extent: ±half_fov around the radar vertical centre.
  const float roi_y_min = v_center - fy_s * tan_half;
  const float roi_y_max = v_center + fy_s * tan_half;

  // Clamp to frame bounds
  const float x0 = std::max(roi_x_min, 0.f);
  const float y0 = std::max(roi_y_min, 0.f);
  const float x1 = std::min(roi_x_max, frame_width);
  const float y1 = std::min(roi_y_max, frame_height);

  return cv::Rect2f(x0, y0, x1 - x0, y1 - y0);
}

std::vector<FusedObject> SensorFusion::fuse(const DetBatch &camera,
                                            const RadarTargets &radar) {
  std::vector<FusedObject> fused;

  if (camera.dets.empty()) {
    return fused;
  }

  // ── Phase 1: IOU Deduplication ──────────────────────────────────────────
  std::vector<Det> deduped_dets;
  std::vector<bool> dropped(camera.dets.size(), false);

  for (size_t i = 0; i < camera.dets.size(); ++i) {
    if (dropped[i])
      continue;
    for (size_t j = i + 1; j < camera.dets.size(); ++j) {
      if (dropped[j])
        continue;

      if (calculateIOU(camera.dets[i].box_px, camera.dets[j].box_px) > 0.75f) {
        if (camera.dets[i].score < camera.dets[j].score ||
            (camera.dets[i].score == camera.dets[j].score &&
             camera.dets[i].object_id > camera.dets[j].object_id)) {
          dropped[i] = true;
          break; // i is dropped, stop inner loop
        } else {
          dropped[j] = true;
        }
      }
    }
    if (!dropped[i]) {
      deduped_dets.push_back(camera.dets[i]);
    }
  }

  std::cout << "[StageE: 1_NMS] Raw Dets: " << camera.dets.size() 
            << " -> Deduped: " << deduped_dets.size() << "\n";

  // Read pitch atomically (written by ZMQ imuThread, read here by viz thread)
  const float theta = pitch_rad_.load(std::memory_order_relaxed);

  const float fw = config_.calib_width_px;
  const float fh = config_.calib_height_px;

  if (fw != cached_roi_width_ || fh != cached_roi_height_) {
    cached_roi_ = computeRadarROI(fw, fh);
    cached_roi_width_ = fw;
    cached_roi_height_ = fh;
  }
  const cv::Rect2f &roi = cached_roi_;

  const float fy_s = config_.f_y;
  const float cy_s = config_.c_y;

  if (g_verbose_mode.load()) {
    std::cout << "[Fusion] Frame " << camera.h.seq << ": "
              << deduped_dets.size() << " dets (deduped), "
              << radar.targets.size() << " radar targets" << "  θ=" << theta
              << " rad\n";
  }

  fused.reserve(deduped_dets.size());

  for (const auto &det : deduped_dets) {

    // ── Phase 2: Camera Distance Estimation ─────────────────────────────
    // Bottom-centre pixel (u=horizontal centre, v=bottom row of bbox)
    const float u = det.box_px.x + det.box_px.width * 0.5f;
    const float v = det.box_px.y + det.box_px.height; // bottom row

    const float z_cam = estimateDistance(v, fy_s, cy_s, theta);
    const bool z_valid = (z_cam >= config_.z_min_m && z_cam <= config_.z_max_m);

    std::cout << "[StageE: 2_Depth] ID " << det.object_id 
              << " | v_bottom: " << v << " | Z_cam: " << z_cam << "m\n";

    // ── Phase 5, Gate 1: Spatial ROI ────────────────────────────────────
    if (!inRadarROI(u, v, roi)) {
      if (g_verbose_mode.load()) {
        std::cout << "  [Fusion] obj " << det.object_id
                  << " DROPPED: outside radar ROI\n";
      }
      continue; // spec: discard
    }

    // Build the FusedObject with camera data regardless of radar match
    FusedObject obj;
    obj.object_id = det.object_id;
    obj.object_class = det.cls;
    obj.score = det.score;
    obj.box_px = det.box_px;
    obj.centroid_px = det.centroid;
    obj.z_cam_m = z_valid ? z_cam : 0.f;
    obj.v_cam_mps = 0.f; // Removed historical V_cam
    obj.sources = SRC_CAM_F;

    // ── Phase 3 & 4: Radar Extrinsic Translation & Gating ───────────────
    // Find the best-matching radar target that passes all distance and
    // spatial gates.  "Best" = minimises |Z_cam - Z_rad_adj|.
    int best_radar_idx = -1;
    float best_dist_delta = std::numeric_limits<float>::max();

    if (z_valid) {
      for (size_t i = 0; i < radar.targets.size(); ++i) {
        const RadarTarget &tgt = radar.targets[i];

        // Step 3: Radar Extrinsic Translation
        float z_rad_adj = tgt.range_m - 0.0127f;
        // Divide by zero protection on z_rad_adj
        if (z_rad_adj < 0.1f)
          continue;

        float v_rad_proj = cy_s + fy_s * (-0.0762f / z_rad_adj);

        std::string gate_result = "FUSED!";

        // Step 4: Distance & Spatial Gating
        const float dz = std::abs(z_cam - z_rad_adj);
        if (dz > 3.0f) {
          gate_result = "REJECTED (Distance dz > 3.0m)";
        } else if (v_rad_proj < det.box_px.y ||
                   v_rad_proj > (det.box_px.y + det.box_px.height)) {
          gate_result = "REJECTED (Spatial Bounds)";
        }

        std::cout << "[StageE: 3_Gate] ID " << det.object_id << " vs Radar | "
                  << "Z_cam: " << z_cam << "m, Z_rad_adj: " << z_rad_adj << "m, dZ: " << dz << "m | "
                  << "v_proj: " << v_rad_proj << " (Bounds: " << det.box_px.y << " to " << (det.box_px.y + det.box_px.height) << ") "
                  << "-> " << gate_result << "\n";

        if (gate_result != "FUSED!") {
          continue;
        }

        // Prefer the closest distance-matched target
        if (dz < best_dist_delta) {
          best_dist_delta = dz;
          best_radar_idx = static_cast<int>(i);
        }
      }
    }

    if (best_radar_idx >= 0) {
      const RadarTarget &tgt = radar.targets[best_radar_idx];

      obj.has_radar = true;
      obj.range_m = tgt.range_m - 0.0127f;
      // Inherit radar velocity (keep spec convention where negative is
      // approaching out of FCW)
      obj.radial_vel_mps = tgt.radial_vel_mps;
      obj.sources |= SRC_RAD_F;

      // TTC authoritative from radar
      obj.ttc_s = computeTTC(obj.range_m, obj.radial_vel_mps);

      if (g_verbose_mode.load()) {
        std::cout << "  [Fusion] MATCHED obj " << det.object_id
                  << " cls=" << det.cls << " Z_cam=" << z_cam
                  << "m Z_fused=" << obj.range_m
                  << "m V_fused=" << obj.radial_vel_mps << "m/s TTC="
                  << (obj.ttc_s < 999.f ? std::to_string((int)obj.ttc_s) + "s"
                                        : "inf")
                  << "\n";
      }
    } else {
      // Camera-only object: no valid radar association
      obj.has_radar = false;
      obj.range_m = 0.f;
      obj.radial_vel_mps = 0.f;
      obj.ttc_s = std::numeric_limits<float>::infinity();
    }

    fused.push_back(obj);
  }

  return fused;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private helpers
// ─────────────────────────────────────────────────────────────────────────────

float SensorFusion::calculateIOU(const cv::Rect2f &a,
                                 const cv::Rect2f &b) const {
  float x_left = std::max(a.x, b.x);
  float y_top = std::max(a.y, b.y);
  float x_right = std::min(a.x + a.width, b.x + b.width);
  float y_bottom = std::min(a.y + a.height, b.y + b.height);

  if (x_right < x_left || y_bottom < y_top) {
    return 0.0f;
  }

  float intersection_area = (x_right - x_left) * (y_bottom - y_top);
  float a_area = a.width * a.height;
  float b_area = b.width * b.height;

  return intersection_area / (a_area + b_area - intersection_area);
}

float SensorFusion::estimateDistance(float v_bottom, float fy_scaled,
                                     float cy_scaled, float pitch_rad) const {
  // α = atan((v - c_y) / f_y)  [vertical optical angle]
  const float alpha = std::atan((v_bottom - cy_scaled) / fy_scaled);
  const float angle = pitch_rad + alpha;

  if (angle <= 0.001f) {
    return 150.0f; // Max Range / Horizon
  }

  return config_.cam_height_m / std::tan(angle);
}

bool SensorFusion::inRadarROI(float u, float v, const cv::Rect2f &roi) const {
  return u >= roi.x && u <= (roi.x + roi.width) && v >= roi.y &&
         v <= (roi.y + roi.height);
}

float SensorFusion::computeTTC(float z_rad, float v_rad_doppler) const {
  if (v_rad_doppler < 0.0f) {
    return z_rad / std::abs(v_rad_doppler);
  }
  return std::numeric_limits<float>::infinity();
}

} // namespace adas
