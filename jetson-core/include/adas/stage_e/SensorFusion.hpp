// File: include/adas/stage_e/SensorFusion.hpp
// Camera-Radar fusion for FCW vertical slice
// Uses a ground-plane pinhole geometry model to estimate longitudinal distance
// from 2D bounding boxes and fuses with FMCW radar using 3-condition gating.
#pragma once

#include "adas/common/Types.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace adas {

// ─────────────────────────────────────────────────────────────────────────────
//                         FUSED OBJECT (Stage E output)
// ─────────────────────────────────────────────────────────────────────────────

/// A single validated, fused object that has passed all gating conditions.
struct FusedObject {
    uint64_t object_id;   ///< Persistent tracker ID from nvtracker (Det::object_id)
    int      object_class;
    float    score;

    // Camera data
    cv::Rect2f  box_px;
    cv::Point2f centroid_px;
    float       z_cam_m;       ///< Ground-plane camera distance estimate (m) [NEW]
    float       v_cam_mps;     ///< Camera-derived longitudinal velocity (m/s) [NEW]
                               ///< Negative = approaching (matches spec convention)

    // Radar data (populated only when all 3 gates pass)
    bool  has_radar;
    float range_m;           ///< Z_rad from radar (authoritative range)
    float radial_vel_mps;    ///< V_rad (negative = approaching, per OPS243-A sign)

    // Output
    float    ttc_s;    ///< Time-to-collision (s). INFINITY if not approaching.
    uint16_t sources;  ///< SensorSource bitmask

    FusedObject()
        : object_id(UINT64_MAX), object_class(0), score(0.f),
          box_px(), centroid_px(),
          z_cam_m(0.f), v_cam_mps(0.f),
          has_radar(false), range_m(0.f), radial_vel_mps(0.f),
          ttc_s(std::numeric_limits<float>::infinity()),
          sources(SRC_NONE) {}
};

// ─────────────────────────────────────────────────────────────────────────────
//                         FUSION CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────

/// All parameters needed for the ground-plane 1D–2D fusion pipeline.
/// Extract from your calibration YAML and rig geometry before constructing
/// SensorFusion. Values set here come from FrontCam_calibration.yaml.
struct FusionConfig {
    // ── Camera intrinsics (FrontCam_calibration.yaml, 1280×720 reference) ──
    float f_x = 828.752f;   ///< Horizontal focal length (pixels)
    float f_y = 829.188f;   ///< Vertical focal length   (pixels)
    float c_x = 606.709f;   ///< Principal point X       (pixels)
    float c_y = 397.742f;   ///< Principal point Y       (pixels)

    // Reference resolution these intrinsics were calibrated at.
    // When the pipeline runs at a different resolution, intrinsics are scaled.
    float calib_width_px  = 1280.f;
    float calib_height_px = 720.f;

    // ── Rig geometry ────────────────────────────────────────────────────────
    float cam_height_m      = 1.30f;    ///< H: camera optical centre above ground (m)
    float radar_below_cam_m = 0.0762f;  ///< Physical distance radar is below camera (m)
                                        ///< = 3 inches. Used to project radar centre
                                        ///< into image space for ROI computation.

    // ── Radar FOV projection ────────────────────────────────────────────────
    float radar_half_fov_deg = 15.f;    ///< Half-angle of 30° radar FOV (degrees)
    float roi_z_ref_m        = 20.f;    ///< Reference depth for vertical ROI centre.
                                        ///< At 20 m, radar_below_cam_m shifts ROI
                                        ///< down by ~4 px — negligible but correct.

    // ── Distance & velocity constraint derivation ───────────────────────────
    float z_min_m = 1.0f;              ///< Reject Z_cam estimates below this (noise)
    float z_max_m = 80.0f;            ///< Reject Z_cam estimates above this

    // ── Gating thresholds (Phase 5) ─────────────────────────────────────────
    float dist_gate_m  = 5.0f;         ///< |Z_cam - Z_rad| < dist_gate_m to match
    float vel_gate_mps = 3.0f;         ///< |V_cam - V_rad| < vel_gate_mps to match

    // ── Track history ────────────────────────────────────────────────────────
    int track_history_len = 8;         ///< Max Z_cam samples retained per track_id

    FusionConfig() = default;
};

// ─────────────────────────────────────────────────────────────────────────────
//                         SENSOR FUSION CLASS
// ─────────────────────────────────────────────────────────────────────────────

/// SensorFusion: Implements the ground-plane 1D–2D fusion pipeline.
///
/// Pipeline overview (per user spec):
///   Phase 1 — Read intrinsics from FusionConfig (static at construction).
///   Phase 2 — Estimate Z_cam from bottom-centre pixel via pinhole geometry.
///   Phase 3 — Estimate V_cam from per-track Z_cam history (Δz/Δt).
///   Phase 4 — Read Z_rad, V_rad from each radar target.
///   Phase 5 — Gate: spatial ROI + distance match + velocity match → FusedObject.
///
/// Thread safety: setPitch() may be called from the ZMQ imuThread at any time;
/// it writes to an atomic<float>. fuse() is called from the visualisation thread
/// only and reads that atomic safely.
class SensorFusion {
  public:
    explicit SensorFusion(const FusionConfig& config = FusionConfig());

    /// Set the current camera pitch angle (radians) received from the Pi.
    /// Called by the ZMQ IMU pitch thread; stored atomically.
    /// Positive θ = nose-up (camera tilted toward horizon).
    void setPitch(float pitch_rad) {
        pitch_rad_.store(pitch_rad, std::memory_order_relaxed);
    }

    /// Get the last pitch value (for logging / diagnostics).
    float getPitch() const {
        return pitch_rad_.load(std::memory_order_relaxed);
    }

    /// Run the full fusion pipeline for one frame.
    /// @param camera  DetBatch from Stage B (DeepStream probe output)
    /// @param radar   RadarTargets from Stage A (OPS243-A FrontRadar)
    /// @return        Validated FusedObjects that passed all gating conditions
    std::vector<FusedObject> fuse(const DetBatch& camera,
                                  const RadarTargets& radar);

    /// Compute the 2D radar ROI bounding box scaled for the given resolution.
    /// Call this once per resolution change; result can be cached by caller.
    cv::Rect2f computeRadarROI(float frame_width, float frame_height) const;

  private:
    // ── Phase 2 ─────────────────────────────────────────────────────────────
    /// Estimate longitudinal distance Z_cam from bottom-centre pixel v using
    /// the ground-plane model:  Z = H / tan(θ + α), where α = atan2(v - c_y, f_y).
    /// Returns zero (invalid) if the geometry produces a negative or implausible Z.
    float estimateDistance(float v_bottom, float fy_scaled, float cy_scaled,
                           float pitch_rad) const;

    /// Helper to compute Intersection over Union of two bounding boxes (Phase 1)
    float calculateIOU(const cv::Rect2f& a, const cv::Rect2f& b) const;


    // ── Phase 5 ─────────────────────────────────────────────────────────────
    /// Return true if the bottom-centre pixel (u, v) is inside the radar ROI.
    bool inRadarROI(float u, float v, const cv::Rect2f& roi) const;

    /// Given Z_rad and V_rad, compute TTC.
    /// Convention: V_rad < 0 means approaching → TTC = Z_rad / |V_rad|.
    float computeTTC(float z_rad, float v_rad_approaching) const;

    // ── State ────────────────────────────────────────────────────────────────
    FusionConfig config_;

    /// Current camera pitch from ZMQ IMU pitch message (atomic for thread safety).
    std::atomic<float> pitch_rad_{0.0f};

    /// Cached ROI for the last seen frame resolution (lazy-updated in fuse()).
    mutable cv::Rect2f cached_roi_;
    mutable float      cached_roi_width_  = -1.f;
    mutable float      cached_roi_height_ = -1.f;
};

} // namespace adas
