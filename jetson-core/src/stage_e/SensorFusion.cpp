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
//   3. |V_cam - V_rad|  < vel_gate_mps    (both normalised to negative=approaching)
//
// TTC = Z_rad / |V_rad|   (radar range is authoritative)
#include "adas/stage_e/SensorFusion.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "adas/common/Globals.hpp"

namespace adas {

SensorFusion::SensorFusion(const FusionConfig& config) : config_(config) {}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────

cv::Rect2f SensorFusion::computeRadarROI(float frame_width,
                                          float frame_height) const {
    // Scale intrinsics from calibration resolution to current resolution.
    // This makes the ROI robust to resolution changes (e.g. 1280×720 → 1024×540).
    const float sx = frame_width  / config_.calib_width_px;
    const float sy = frame_height / config_.calib_height_px;

    const float fx_s = config_.f_x * sx;
    const float fy_s = config_.f_y * sy;
    const float cx_s = config_.c_x * sx;
    const float cy_s = config_.c_y * sy;

    const float tan_half = std::tan(config_.radar_half_fov_deg * (float)M_PI / 180.f);

    // Horizontal extent: ±half_fov from camera principal point.
    // Camera and radar share the same horizontal boresight axis.
    const float roi_x_min = cx_s - fx_s * tan_half;
    const float roi_x_max = cx_s + fx_s * tan_half;

    // Vertical centre: the radar is physically radar_below_cam_m below the
    // camera.  At reference depth roi_z_ref_m, this translates to a downward
    // pixel offset of:  Δv = fy_s * radar_below_cam_m / roi_z_ref_m
    // For roi_z_ref_m=20m: Δv ≈ 829*0.0762/20 ≈ 3.2 px — negligible but correct.
    const float v_center = cy_s +
                           fy_s * (config_.radar_below_cam_m / config_.roi_z_ref_m);

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

std::vector<FusedObject> SensorFusion::fuse(const DetBatch&    camera,
                                             const RadarTargets& radar) {
    std::vector<FusedObject> fused;

    if (camera.dets.empty()) {
        return fused;
    }

    // Read pitch atomically (written by ZMQ imuThread, read here by viz thread)
    const float theta = pitch_rad_.load(std::memory_order_relaxed);

    // ── Lazy-cache the ROI for the current frame resolution ──────────────────
    // Infer frame size from the first detection's bounding box context.
    // We use hardcoded calibration width/height as the canonical reference;
    // if the pipeline is running at a different resolution, DeepStream's
    // bounding boxes will be in that resolution's pixel space.  Detect the
    // active resolution from the config (we assume the pipeline sends bboxes
    // in calib_width × calib_height space unless the caller overrides).
    // For full robustness with resolution changes: the caller can pre-compute
    // the ROI via computeRadarROI(frame_w, frame_h) and use a batch header
    // field — but here we default to calibration resolution as that is what
    // the DeepStream pipeline is currently configured for.
    const float fw = config_.calib_width_px;
    const float fh = config_.calib_height_px;

    if (fw != cached_roi_width_ || fh != cached_roi_height_) {
        cached_roi_        = computeRadarROI(fw, fh);
        cached_roi_width_  = fw;
        cached_roi_height_ = fh;
    }
    const cv::Rect2f& roi = cached_roi_;

    // Scale intrinsics (identity when running at calibration resolution)
    const float sx   = fw / config_.calib_width_px;
    const float sy   = fh / config_.calib_height_px;
    const float fy_s = config_.f_y * sy;
    const float cy_s = config_.c_y * sy;

    // Log fusion frame header once when verbose
    if (g_verbose_mode.load()) {
        std::cout << "[Fusion] Frame " << camera.h.seq
                  << ": " << camera.dets.size() << " dets, "
                  << radar.targets.size() << " radar targets"
                  << "  θ=" << theta << " rad\n";
    }

    fused.reserve(camera.dets.size());

    const uint64_t t_ns = camera.h.t_ingest_ns;

    for (const auto& det : camera.dets) {

        // ── Phase 2: Camera Distance Estimation ─────────────────────────────
        // Bottom-centre pixel (u=horizontal centre, v=bottom row of bbox)
        const float u = det.box_px.x + det.box_px.width  * 0.5f;
        const float v = det.box_px.y + det.box_px.height;       // bottom row

        const float z_cam = estimateDistance(v, fy_s, cy_s, theta);
        const bool  z_valid = (z_cam >= config_.z_min_m && z_cam <= config_.z_max_m);

        // ── Phase 3: Camera Velocity Estimation ──────────────────────────────
        float v_cam = std::numeric_limits<float>::quiet_NaN();
        if (z_valid) {
            v_cam = updateAndEstimateVelocity(det.object_id, z_cam, t_ns);
        }

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
        obj.object_id    = det.object_id;
        obj.object_class = det.cls;
        obj.score        = det.score;
        obj.box_px       = det.box_px;
        obj.centroid_px  = det.centroid;
        obj.z_cam_m      = z_valid ? z_cam : 0.f;
        obj.v_cam_mps    = std::isnan(v_cam) ? 0.f : v_cam;
        obj.sources      = SRC_CAM_F;

        // ── Phase 4 + Phase 5, Gates 2 & 3: Radar Association ───────────────
        // Find the best-matching radar target that passes all distance and
        // velocity gates.  "Best" = minimises |Z_cam - Z_rad|.
        int   best_radar_idx  = -1;
        float best_dist_delta = std::numeric_limits<float>::max();

        for (size_t i = 0; i < radar.targets.size(); ++i) {
            const RadarTarget& tgt = radar.targets[i];

            const float z_rad = tgt.range_m;
            // OPS243-A sign convention: positive radial_vel = approaching.
            // Spec normalises to negative=approaching; flip sign for V_cam comparison.
            // V_rad_norm < 0 means approaching (matches V_cam convention).
            const float v_rad_norm = -tgt.radial_vel_mps;

            // Gate 2: distance match
            if (!z_valid) {
                // No valid Z_cam — skip velocity & distance gates
                // (camera-only object, no radar association possible)
                break;
            }
            const float dz = std::abs(z_cam - z_rad);
            if (dz >= config_.dist_gate_m) {
                continue;
            }

            // Gate 3: velocity match
            // If V_cam is not yet available (<2 history points), skip velocity
            // gate — allow distance-only match for the first few frames of a track.
            if (!std::isnan(v_cam)) {
                const float dv = std::abs(v_cam - v_rad_norm);
                if (dv >= config_.vel_gate_mps) {
                    continue;
                }
            }

            // Prefer the closest distance-matched target
            if (dz < best_dist_delta) {
                best_dist_delta = dz;
                best_radar_idx  = static_cast<int>(i);
            }
        }

        if (best_radar_idx >= 0) {
            const RadarTarget& tgt = radar.targets[best_radar_idx];

            obj.has_radar      = true;
            obj.range_m        = tgt.range_m;
            // Expose radar velocity in the spec convention (positive=approaching)
            // so the existing FCWMonitor logic is unchanged.
            obj.radial_vel_mps = tgt.radial_vel_mps;
            obj.sources       |= SRC_RAD_F;

            // TTC authoritative from radar (Z_rad / |V_rad|, positive approach)
            obj.ttc_s = computeTTC(tgt.range_m, tgt.radial_vel_mps);

            if (g_verbose_mode.load()) {
                std::cout << "  [Fusion] MATCHED obj " << det.object_id
                          << " cls=" << det.cls
                          << " Z_cam=" << z_cam << "m V_cam=" << obj.v_cam_mps
                          << "m/s Z_rad=" << tgt.range_m
                          << "m V_rad=" << tgt.radial_vel_mps
                          << "m/s TTC=" << (obj.ttc_s < 999.f
                                             ? std::to_string((int)obj.ttc_s) + "s"
                                             : "inf")
                          << "\n";
            }
        } else {
            // Camera-only object: no valid radar association
            obj.has_radar      = false;
            obj.range_m        = 0.f;
            obj.radial_vel_mps = 0.f;
            obj.ttc_s          = std::numeric_limits<float>::infinity();
        }

        fused.push_back(obj);
    }

    // Evict stale tracks to prevent unbounded memory growth.
    // Keep only track IDs that appeared in this frame.
    {
        std::vector<uint64_t> active_ids;
        active_ids.reserve(camera.dets.size());
        for (const auto& det : camera.dets) {
            active_ids.push_back(det.object_id);
        }
        for (auto it = track_history_.begin(); it != track_history_.end(); ) {
            bool is_active = false;
            for (uint64_t id : active_ids) {
                if (id == it->first) { is_active = true; break; }
            }
            it = is_active ? std::next(it) : track_history_.erase(it);
        }
    }

    return fused;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private helpers
// ─────────────────────────────────────────────────────────────────────────────

float SensorFusion::estimateDistance(float v_bottom,
                                      float fy_scaled,
                                      float cy_scaled,
                                      float pitch_rad) const {
    // α = atan2(v - c_y, f_y)  [vertical optical angle, positive = below horizon]
    const float alpha = std::atan2(v_bottom - cy_scaled, fy_scaled);
    const float angle = pitch_rad + alpha;

    // Avoid division by zero or negative tangent (object behind or at horizon)
    const float tan_angle = std::tan(angle);
    if (tan_angle <= 1e-4f) {
        return 0.f; // Invalid — object at or above horizon in camera frame
    }

    return config_.cam_height_m / tan_angle;
}

float SensorFusion::updateAndEstimateVelocity(uint64_t object_id,
                                               float    z_now,
                                               uint64_t t_ns) {
    auto& hist = track_history_[object_id];

    // Append current sample
    hist.push_back({z_now, t_ns});

    // Enforce history length cap
    while (static_cast<int>(hist.size()) > config_.track_history_len) {
        hist.pop_front();
    }

    // Need at least 2 samples to estimate velocity
    if (hist.size() < 2) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    // Finite-difference over the full available window for noise robustness.
    // V_cam = (Z(t_latest) - Z(t_oldest)) / Δt
    // Negative V_cam = range decreasing = approaching (matches spec).
    const auto& oldest = hist.front();
    const auto& latest = hist.back();

    const float dt_s = static_cast<float>(latest.second - oldest.second) * 1e-9f;
    if (dt_s < 1e-3f) {
        return std::numeric_limits<float>::quiet_NaN(); // Too close in time
    }

    return (latest.first - oldest.first) / dt_s;
}

bool SensorFusion::inRadarROI(float u, float v, const cv::Rect2f& roi) const {
    return u >= roi.x && u <= (roi.x + roi.width)
        && v >= roi.y && v <= (roi.y + roi.height);
}

float SensorFusion::computeTTC(float z_rad, float v_rad_positive_approaching) const {
    // OPS243-A: positive radial_vel_mps = approaching target.
    // TTC only meaningful when target is closing at > 0.1 m/s.
    if (v_rad_positive_approaching > 0.1f) {
        return z_rad / v_rad_positive_approaching;
    }
    return std::numeric_limits<float>::infinity();
}

} // namespace adas
