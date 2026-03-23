// File: include/adas/stage_e/SensorFusion.hpp
// Asynchronous camera-radar fusion with EKF track state for FCW.
#pragma once

#include "adas/common/Types.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace adas {

struct FusedObject {
  uint64_t object_id;
  int object_class; // Canonical ObjectClass value
  float score;

  // Camera data
  cv::Rect2f box_px;
  cv::Point2f centroid_px;
  float z_cam_m;
  float v_cam_mps;

  // Radar data (authoritative longitudinal state when available)
  bool has_radar;
  float range_m;
  float radial_vel_mps; // +toward/inward, -away/outward
  float x_lateral_m;
  float fusion_quality; // [0..1]
  bool speed_fresh;
  uint32_t speed_age_ms;

  // FCW/BEV metadata
  float ttc_s;
  uint16_t sources;
  float theta_rad;
  uint32_t camera_age_ms;
  uint32_t radar_age_ms;
  bool is_predicted_camera;
  bool is_aggressive_mode;

  FusedObject()
      : object_id(UINT64_MAX),
        object_class(static_cast<int>(ObjectClass::Unknown)), score(0.0f),
        box_px(),
        centroid_px(), z_cam_m(0.0f), v_cam_mps(0.0f), has_radar(false),
        range_m(0.0f), radial_vel_mps(0.0f), x_lateral_m(0.0f),
        fusion_quality(0.0f), speed_fresh(false), speed_age_ms(0),
        ttc_s(std::numeric_limits<float>::infinity()), sources(SRC_NONE),
        theta_rad(0.0f), camera_age_ms(0), radar_age_ms(0),
        is_predicted_camera(false), is_aggressive_mode(false) {}
};

struct FusionConfig {
  // Camera intrinsics at calibration resolution.
  float f_x = 828.752f;
  float f_y = 829.188f;
  float c_x = 606.709f;
  float c_y = 397.742f;
  float calib_width_px = 1280.0f;
  float calib_height_px = 720.0f;

  // Rig geometry.
  float cam_height_m = 1.30f;
  float radar_below_cam_m = 0.0762f;

  // Front radar/camera shared intake and range tuning.
  float radar_half_fov_deg = 15.0f; // 30 deg total
  float roi_z_ref_m = 20.0f;
  float z_min_m = 1.0f;
  float z_max_m = 80.0f;

  // Legacy/default range gate baseline.
  float dist_gate_m = 5.5f;

  // FCW v3 association controls.
  float ttc_aggressive_s = 3.0f;
  float normal_angle_gate_deg = 12.5f;
  float aggressive_angle_gate_deg = 18.0f;
  float normal_range_gate_m = 5.5f;
  float aggressive_range_scale = 1.5f;
  uint32_t camera_hold_ms = 400;

  // Association stability guards.
  float max_backward_jump_m = 0.8f; // Reject range jumps away while closing.
  float closing_speed_for_backward_guard_mps = 0.4f;
  float camera_consistency_min_m = 1.25f;
  float camera_consistency_ratio = 0.45f;
  float camera_consistency_penalty = 1.5f;

  // Track retention and output freshness.
  uint32_t track_cleanup_ms = 1600;
  uint32_t radar_hold_ms = 1000;
  uint32_t predicted_camera_threshold_ms = 80;

  // EKF process noise.
  float ekf_q_z = 1.0f;
  float ekf_q_vz = 1.2f;
  float ekf_q_theta = 0.04f;
  float ekf_q_theta_dot = 0.06f;

  // EKF measurement noise.
  float ekf_r_radar_z = 0.35f;
  float ekf_r_radar_vz = 0.55f;
  float ekf_r_cam_theta = 0.018f;
  float ekf_r_cam_z_weak = 30.0f; // weak consistency only

  // Fixed radar range tx offset used historically in this project.
  float radar_tx_m = 0.0127f;

  FusionConfig() = default;
};

class SensorFusion {
public:
  explicit SensorFusion(const FusionConfig &config = FusionConfig());

  void setPitch(float pitch_rad) {
    pitch_rad_.store(pitch_rad, std::memory_order_relaxed);
  }

  float getPitch() const {
    return pitch_rad_.load(std::memory_order_relaxed);
  }

  void setCameraHeight(float height_m) {
    cam_height_m_.store(height_m, std::memory_order_relaxed);
  }

  float getCameraHeight() const {
    return cam_height_m_.load(std::memory_order_relaxed);
  }

  void setCameraHoldMs(uint32_t camera_hold_ms);
  void setRadarHoldMs(uint32_t radar_hold_ms);
  void setTrackCleanupMs(uint32_t track_cleanup_ms);
  void setPredictedCameraThresholdMs(uint32_t predicted_camera_threshold_ms);

  // New asynchronous API.
  void ingestRadar(const RadarTargets &radar, uint64_t now_ns = 0);
  void ingestCamera(const DetBatch &camera, uint64_t now_ns = 0);
  std::vector<FusedObject> getFusedObjects(uint64_t now_ns = 0);

  // Compatibility wrapper for older call sites.
  std::vector<FusedObject> fuse(const DetBatch &camera, const RadarTargets &radar);

  cv::Rect2f computeRadarROI(float frame_width, float frame_height) const;

private:
  struct TrackState {
    uint64_t object_id = UINT64_MAX;
    int object_class = static_cast<int>(ObjectClass::Unknown);
    float score = 0.0f;

    cv::Rect2f box_px;
    cv::Point2f centroid_px;
    float z_cam_m = 0.0f;
    float cam_theta_rad = 0.0f;
    bool has_camera_obs = false;

    // EKF state: [z, vz, theta, theta_dot]
    std::array<float, 4> x{0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 16> P{0.0f}; // row-major 4x4
    bool initialized = false;

    float fusion_quality = 0.0f;
    bool speed_fresh = false;
    uint32_t speed_age_ms = 0;
    bool is_aggressive_mode = false;

    uint64_t last_predict_ns = 0;
    uint64_t last_camera_ns = 0;
    uint64_t last_radar_ns = 0;
    uint64_t last_fused_ns = 0;
  };

  struct RadarObs {
    float range_m = 0.0f;
    float radial_vel_mps = 0.0f;
    bool speed_fresh = false;
    uint32_t speed_age_ms = 0;
  };

  // Core math helpers.
  float estimateDistance(float v_bottom, float fy_scaled, float cy_scaled,
                         float pitch_rad, float cam_height_m) const;
  float calculateIOU(const cv::Rect2f &a, const cv::Rect2f &b) const;
  bool inRadarROI(float u, float v, const cv::Rect2f &roi) const;
  float computeTTC(float z_rad, float v_rad_approaching) const;

  // EKF helpers.
  static float normalizeAngle(float rad);
  void initializeTrack(TrackState &track, const Det &det, float theta_rad,
                       float z_cam_m, uint64_t now_ns);
  void predictTrackTo(TrackState &track, uint64_t now_ns);
  void updateTrackCamera(TrackState &track, float theta_rad, float z_cam_m);
  void updateTrackRadar(TrackState &track, float z_rad_m, bool has_speed,
                        float v_rad_mps);
  void maybeCleanupTracks(uint64_t now_ns);

  FusionConfig config_;

  std::atomic<float> pitch_rad_{0.0f};
  std::atomic<float> cam_height_m_{1.30f};

  mutable cv::Rect2f cached_roi_;
  mutable float cached_roi_width_ = -1.0f;
  mutable float cached_roi_height_ = -1.0f;

  std::unordered_map<uint64_t, TrackState> tracks_;
  RadarTargets latest_radar_;

  std::mutex mutex_;
};

} // namespace adas
