// File: src/stage_e/SensorFusion.cpp
// Asynchronous camera-radar fusion with EKF track state for FCW v3.
#include "adas/stage_e/SensorFusion.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "adas/common/Clock.hpp"
#include "adas/common/Globals.hpp"

namespace adas {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float degToRad(float deg) { return deg * kPi / 180.0f; }

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

} // namespace

SensorFusion::SensorFusion(const FusionConfig &config)
    : config_(config), cam_height_m_(config.cam_height_m) {}

cv::Rect2f SensorFusion::computeRadarROI(float frame_width,
                                         float frame_height) const {
  const float sx = frame_width / config_.calib_width_px;
  const float sy = frame_height / config_.calib_height_px;

  const float fx_s = config_.f_x * sx;
  const float fy_s = config_.f_y * sy;
  const float cx_s = config_.c_x * sx;
  const float cy_s = config_.c_y * sy;

  const float tan_half = std::tan(config_.radar_half_fov_deg * kPi / 180.0f);

  const float roi_x_min = cx_s - fx_s * tan_half;
  const float roi_x_max = cx_s + fx_s * tan_half;

  const float v_center =
      cy_s + fy_s * (config_.radar_below_cam_m / config_.roi_z_ref_m);
  const float roi_y_min = v_center - fy_s * tan_half;
  const float roi_y_max = v_center + fy_s * tan_half;

  const float x0 = std::max(roi_x_min, 0.0f);
  const float y0 = std::max(roi_y_min, 0.0f);
  const float x1 = std::min(roi_x_max, frame_width);
  const float y1 = std::min(roi_y_max, frame_height);

  return cv::Rect2f(x0, y0, x1 - x0, y1 - y0);
}

std::vector<FusedObject> SensorFusion::fuse(const DetBatch &camera,
                                            const RadarTargets &radar) {
  const uint64_t now_ns =
      (camera.h.t_ingest_ns > 0) ? camera.h.t_ingest_ns : Clock::now_ns();
  ingestCamera(camera, now_ns);
  ingestRadar(radar, now_ns);
  return getFusedObjects(now_ns);
}

void SensorFusion::ingestCamera(const DetBatch &camera, uint64_t now_ns) {
  if (camera.dets.empty()) {
    return;
  }

  if (now_ns == 0) {
    now_ns =
        (camera.h.t_ingest_ns > 0) ? camera.h.t_ingest_ns : Clock::now_ns();
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const float fw = config_.calib_width_px;
  const float fh = config_.calib_height_px;
  if (fw != cached_roi_width_ || fh != cached_roi_height_) {
    cached_roi_ = computeRadarROI(fw, fh);
    cached_roi_width_ = fw;
    cached_roi_height_ = fh;
  }
  const cv::Rect2f &roi = cached_roi_;

  // Dedupe overlapping boxes by confidence.
  std::vector<Det> deduped;
  std::vector<bool> dropped(camera.dets.size(), false);
  for (size_t i = 0; i < camera.dets.size(); ++i) {
    if (dropped[i]) {
      continue;
    }
    for (size_t j = i + 1; j < camera.dets.size(); ++j) {
      if (dropped[j]) {
        continue;
      }
      if (calculateIOU(camera.dets[i].box_px, camera.dets[j].box_px) > 0.75f) {
        if (camera.dets[i].score < camera.dets[j].score ||
            (camera.dets[i].score == camera.dets[j].score &&
             camera.dets[i].object_id > camera.dets[j].object_id)) {
          dropped[i] = true;
          break;
        }
        dropped[j] = true;
      }
    }
    if (!dropped[i]) {
      deduped.push_back(camera.dets[i]);
    }
  }

  if (g_verbose_mode.load()) {
    std::cout << "[StageE: 1_NMS] Raw Dets: " << camera.dets.size()
              << " -> Deduped: " << deduped.size() << "\n";
  }

  const float theta_pitch = pitch_rad_.load(std::memory_order_relaxed);
  const float cam_height = cam_height_m_.load(std::memory_order_relaxed);

  for (const auto &det : deduped) {
    if (det.object_id == UINT64_MAX) {
      continue;
    }

    const float u = det.box_px.x + det.box_px.width * 0.5f;
    const float v = det.box_px.y + det.box_px.height;

    if (!inRadarROI(u, v, roi)) {
      continue;
    }

    const float theta = std::atan((det.centroid.x - config_.c_x) / config_.f_x);
    const float z_cam =
        estimateDistance(v, config_.f_y, config_.c_y, theta_pitch, cam_height);

    if (g_verbose_mode.load()) {
      std::cout << "[StageE: 2_Depth] ID " << det.object_id
                << " | v_bottom: " << v << " | Z_cam: " << z_cam << "m\n";
    }

    TrackState &track = tracks_[det.object_id];
    predictTrackTo(track, now_ns);
    if (!track.initialized) {
      initializeTrack(track, det, theta, z_cam, now_ns);
    } else {
      updateTrackCamera(track, theta, z_cam);
    }

    track.object_id = det.object_id;
    track.object_class = det.cls;
    track.score = det.score;
    track.box_px = det.box_px;
    track.centroid_px = det.centroid;
    track.z_cam_m = z_cam;
    track.cam_theta_rad = theta;
    track.has_camera_obs = true;
    track.last_camera_ns = now_ns;
    track.last_fused_ns = now_ns;

    if (g_verbose_mode.load()) {
      std::cout << "[StageE: 2_CamUpdate] ID " << det.object_id
                << " | theta: " << theta << " | z_cam: " << z_cam << "m\n";
    }
  }

  maybeCleanupTracks(now_ns);
}

void SensorFusion::ingestRadar(const RadarTargets &radar, uint64_t now_ns) {
  if (now_ns == 0) {
    now_ns = (radar.h.t_ingest_ns > 0) ? radar.h.t_ingest_ns : Clock::now_ns();
  }

  std::lock_guard<std::mutex> lock(mutex_);

  latest_radar_ = radar;

  if (tracks_.empty() || radar.targets.empty()) {
    maybeCleanupTracks(now_ns);
    return;
  }

  std::vector<RadarObs> radar_obs;
  radar_obs.reserve(radar.targets.size());
  for (const auto &tgt : radar.targets) {
    const float z_adj = tgt.range_m - config_.radar_tx_m;
    if (z_adj < 0.1f) {
      continue;
    }
    RadarObs obs;
    obs.range_m = z_adj;
    obs.radial_vel_mps = tgt.radial_vel_mps;
    obs.speed_fresh = tgt.speed_fresh;
    obs.speed_age_ms = tgt.speed_age_ms;
    radar_obs.push_back(obs);
  }

  if (radar_obs.empty()) {
    maybeCleanupTracks(now_ns);
    return;
  }

  std::vector<bool> claimed(radar_obs.size(), false);

  for (auto &[id, track] : tracks_) {
    (void)id;
    if (!track.initialized) {
      continue;
    }

    predictTrackTo(track, now_ns);

    const float ttc_pred = computeTTC(track.x[0], track.x[1]);
    const bool aggressive =
        std::isfinite(ttc_pred) && ttc_pred < config_.ttc_aggressive_s;
    track.is_aggressive_mode = aggressive;

    const float angle_gate_deg = aggressive ? config_.aggressive_angle_gate_deg
                                            : config_.normal_angle_gate_deg;
    const float angle_gate_rad = degToRad(angle_gate_deg);

    if (std::abs(track.x[2]) > angle_gate_rad) {
      continue;
    }

    const float range_gate_m =
        config_.normal_range_gate_m *
        (aggressive ? config_.aggressive_range_scale : 1.0f);

    int best_idx = -1;
    float best_nis = std::numeric_limits<float>::max();

    for (size_t i = 0; i < radar_obs.size(); ++i) {
      if (claimed[i]) {
        continue;
      }

      const auto &obs = radar_obs[i];
      const float dz = obs.range_m - track.x[0];
      if (std::abs(dz) > range_gate_m) {
        continue;
      }

      // Prevent sudden range jumps away from the sensor when the tracked target
      // is already closing in. This rejects common "target swap" regressions.
      if (track.last_radar_ns > 0 &&
          track.x[1] > config_.closing_speed_for_backward_guard_mps &&
          dz > config_.max_backward_jump_m) {
        if (g_verbose_mode.load()) {
          std::cout << "[StageE: 3_GateReject] ID " << track.object_id
                    << " | reason: backward_jump" << " | Z_pred: " << track.x[0]
                    << "m" << " | Z_rad: " << obs.range_m << "m"
                    << " | dZ: " << dz << "m"
                    << " | max_backtrack: " << config_.max_backward_jump_m
                    << "m\n";
        }
        continue;
      }

      const float sz = std::max(track.P[0] + config_.ekf_r_radar_z, 1e-3f);
      float nis = (dz * dz) / sz;

      if (obs.speed_fresh) {
        const float dv = obs.radial_vel_mps - track.x[1];
        const float sv = std::max(track.P[5] + config_.ekf_r_radar_vz, 1e-3f);
        nis += (dv * dv) / sv;
      }

      const float theta_res = normalizeAngle(track.cam_theta_rad - track.x[2]);
      const float stheta =
          std::max(track.P[10] + config_.ekf_r_cam_theta, 1e-4f);
      nis += 0.25f * (theta_res * theta_res) / stheta;

      const bool cam_recent_for_consistency =
          track.last_camera_ns > 0 && now_ns >= track.last_camera_ns &&
          (now_ns - track.last_camera_ns) <=
              static_cast<uint64_t>(config_.camera_hold_ms) * 1000000ULL &&
          track.z_cam_m > 0.1f;
      if (cam_recent_for_consistency) {
        const float d_cam = std::abs(obs.range_m - track.z_cam_m);
        const float cam_gate =
            std::max(config_.camera_consistency_min_m,
                     config_.camera_consistency_ratio * track.z_cam_m);
        if (cam_gate > 1e-3f) {
          const float cam_norm = d_cam / cam_gate;
          nis += config_.camera_consistency_penalty * cam_norm * cam_norm;
        }
      }

      if (nis < best_nis) {
        best_nis = nis;
        best_idx = static_cast<int>(i);
      }
    }

    if (best_idx < 0) {
      continue;
    }

    claimed[best_idx] = true;
    const RadarObs &best = radar_obs[best_idx];
    const float z_pred_before_update = track.x[0];

    updateTrackRadar(track, best.range_m, best.speed_fresh,
                     best.radial_vel_mps);

    track.last_radar_ns = now_ns;
    track.last_fused_ns = now_ns;
    track.speed_fresh = best.speed_fresh;
    track.speed_age_ms = best.speed_age_ms;
    track.fusion_quality = clamp01(1.0f / (1.0f + best_nis));

    if (g_verbose_mode.load()) {
      const float d_cam = (track.z_cam_m > 0.1f)
                              ? std::abs(best.range_m - track.z_cam_m)
                              : -1.0f;
      std::cout << "[StageE: 3_Gate] ID " << track.object_id
                << " vs Radar | Z_cam: " << track.z_cam_m << "m"
                << ", Z_rad_adj: " << best.range_m << "m"
                << ", dZ_cam: " << d_cam << "m"
                << " | Z_pred: " << z_pred_before_update << "m"
                << " | V: " << best.radial_vel_mps << "m/s"
                << " | score(NIS): " << best_nis
                << (aggressive ? " [AGGR]" : "") << "\n";
    }
  }

  maybeCleanupTracks(now_ns);
}

std::vector<FusedObject> SensorFusion::getFusedObjects(uint64_t now_ns) {
  if (now_ns == 0) {
    now_ns = Clock::now_ns();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  maybeCleanupTracks(now_ns);

  std::vector<FusedObject> out;
  out.reserve(tracks_.size());

  for (auto &[id, track] : tracks_) {
    (void)id;
    if (!track.initialized) {
      continue;
    }

    predictTrackTo(track, now_ns);

    const uint32_t cam_age_ms =
        (track.last_camera_ns > 0 && now_ns >= track.last_camera_ns)
            ? static_cast<uint32_t>((now_ns - track.last_camera_ns) /
                                    1000000ULL)
            : std::numeric_limits<uint32_t>::max();

    const uint32_t radar_age_ms =
        (track.last_radar_ns > 0 && now_ns >= track.last_radar_ns)
            ? static_cast<uint32_t>((now_ns - track.last_radar_ns) / 1000000ULL)
            : std::numeric_limits<uint32_t>::max();

    const bool cam_recent = cam_age_ms <= config_.track_cleanup_ms;
    const bool radar_recent = radar_age_ms <= config_.radar_hold_ms;

    if (!cam_recent && !radar_recent) {
      continue;
    }

    FusedObject obj;
    obj.object_id = track.object_id;
    obj.object_class = track.object_class;
    obj.score = track.score;
    obj.box_px = track.box_px;
    obj.centroid_px = track.centroid_px;
    obj.z_cam_m = track.z_cam_m;
    obj.v_cam_mps = 0.0f;

    obj.theta_rad = track.x[2];
    obj.camera_age_ms = cam_age_ms;
    obj.radar_age_ms = radar_age_ms;
    obj.is_predicted_camera =
        cam_age_ms > config_.predicted_camera_threshold_ms &&
        cam_age_ms < std::numeric_limits<uint32_t>::max();
    obj.is_aggressive_mode = track.is_aggressive_mode;

    obj.sources = SRC_NONE;
    if (cam_recent) {
      obj.sources |= SRC_CAM_F;
    }

    if (radar_recent) {
      obj.has_radar = true;
      obj.range_m = std::max(track.x[0], 0.0f);
      obj.radial_vel_mps = track.x[1];
      obj.x_lateral_m = obj.range_m * std::tan(obj.theta_rad);
      obj.fusion_quality = track.fusion_quality;
      obj.speed_age_ms =
          (radar_age_ms == std::numeric_limits<uint32_t>::max())
              ? track.speed_age_ms
              : static_cast<uint32_t>(track.speed_age_ms + radar_age_ms);
      obj.speed_fresh =
          track.speed_fresh && (obj.speed_age_ms <= config_.radar_hold_ms);
      obj.ttc_s = computeTTC(obj.range_m, obj.radial_vel_mps);
      obj.sources |= SRC_RAD_F;
    } else {
      obj.has_radar = false;
      obj.range_m = 0.0f;
      obj.radial_vel_mps = 0.0f;
      obj.x_lateral_m = (track.z_cam_m > 0.1f)
                            ? (track.z_cam_m * std::tan(obj.theta_rad))
                            : 0.0f;
      obj.fusion_quality = 0.0f;
      obj.speed_fresh = false;
      obj.speed_age_ms = 0;
      obj.ttc_s = std::numeric_limits<float>::infinity();
    }

    out.push_back(obj);
  }

  return out;
}

float SensorFusion::normalizeAngle(float rad) {
  while (rad > kPi) {
    rad -= 2.0f * kPi;
  }
  while (rad < -kPi) {
    rad += 2.0f * kPi;
  }
  return rad;
}

void SensorFusion::initializeTrack(TrackState &track, const Det &det,
                                   float theta_rad, float z_cam_m,
                                   uint64_t now_ns) {
  track.object_id = det.object_id;
  track.object_class = det.cls;
  track.score = det.score;
  track.box_px = det.box_px;
  track.centroid_px = det.centroid;
  track.z_cam_m = z_cam_m;
  track.cam_theta_rad = theta_rad;
  track.has_camera_obs = true;

  const float z_init =
      (z_cam_m >= config_.z_min_m && z_cam_m <= config_.z_max_m) ? z_cam_m
                                                                 : 20.0f;

  track.x = {z_init, 0.0f, theta_rad, 0.0f};

  track.P.fill(0.0f);
  track.P[0] = 25.0f;
  track.P[5] = 9.0f;
  track.P[10] = 0.30f;
  track.P[15] = 0.20f;

  track.initialized = true;
  track.last_predict_ns = now_ns;
  track.last_camera_ns = now_ns;
  track.last_fused_ns = now_ns;
}

void SensorFusion::predictTrackTo(TrackState &track, uint64_t now_ns) {
  if (!track.initialized) {
    return;
  }
  if (track.last_predict_ns == 0) {
    track.last_predict_ns = now_ns;
    return;
  }
  if (now_ns <= track.last_predict_ns) {
    return;
  }

  float dt =
      static_cast<float>(Clock::ns_to_sec(now_ns - track.last_predict_ns));
  dt = std::clamp(dt, 0.0f, 0.25f);
  if (dt <= 0.0f) {
    track.last_predict_ns = now_ns;
    return;
  }

  // x = F x, with F based on constant velocity/turn-rate model.
  std::array<float, 4> x_new = track.x;
  x_new[0] = track.x[0] + dt * track.x[1];
  x_new[1] = track.x[1];
  x_new[2] = normalizeAngle(track.x[2] + dt * track.x[3]);
  x_new[3] = track.x[3];

  // P = F P F' + Q
  float F[16] = {1.0f, dt,   0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 1.0f, dt,   0.0f, 0.0f, 0.0f, 1.0f};

  float FP[16] = {0.0f};
  float FPFt[16] = {0.0f};

  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      for (int k = 0; k < 4; ++k) {
        FP[r * 4 + c] += F[r * 4 + k] * track.P[k * 4 + c];
      }
    }
  }

  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      for (int k = 0; k < 4; ++k) {
        FPFt[r * 4 + c] += FP[r * 4 + k] * F[c * 4 + k];
      }
    }
  }

  float q[4] = {config_.ekf_q_z * dt * dt, config_.ekf_q_vz * dt,
                config_.ekf_q_theta * dt * dt, config_.ekf_q_theta_dot * dt};
  for (int i = 0; i < 4; ++i) {
    FPFt[i * 4 + i] += q[i];
  }

  track.x = x_new;
  for (int i = 0; i < 16; ++i) {
    track.P[i] = FPFt[i];
  }
  track.last_predict_ns = now_ns;
}

void SensorFusion::updateTrackCamera(TrackState &track, float theta_rad,
                                     float z_cam_m) {
  if (!track.initialized) {
    return;
  }

  // Scalar update on theta component (index=2).
  {
    const int idx = 2;
    const float y = normalizeAngle(theta_rad - track.x[idx]);
    const float s =
        std::max(track.P[idx * 4 + idx] + config_.ekf_r_cam_theta, 1e-5f);

    float K[4] = {0.0f};
    for (int r = 0; r < 4; ++r) {
      K[r] = track.P[r * 4 + idx] / s;
      track.x[r] += K[r] * y;
    }
    track.x[2] = normalizeAngle(track.x[2]);

    float P_old[16];
    for (int i = 0; i < 16; ++i) {
      P_old[i] = track.P[i];
    }

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        track.P[r * 4 + c] = P_old[r * 4 + c] - K[r] * P_old[idx * 4 + c];
      }
    }
  }

  // Weak scalar consistency update on z_cam (high measurement noise).
  if (z_cam_m >= config_.z_min_m && z_cam_m <= config_.z_max_m) {
    const int idx = 0;
    const float y = z_cam_m - track.x[idx];
    const float s =
        std::max(track.P[idx * 4 + idx] + config_.ekf_r_cam_z_weak, 1e-5f);

    float K[4] = {0.0f};
    for (int r = 0; r < 4; ++r) {
      K[r] = track.P[r * 4 + idx] / s;
      track.x[r] += K[r] * y;
    }

    float P_old[16];
    for (int i = 0; i < 16; ++i) {
      P_old[i] = track.P[i];
    }

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        track.P[r * 4 + c] = P_old[r * 4 + c] - K[r] * P_old[idx * 4 + c];
      }
    }
  }
}

void SensorFusion::updateTrackRadar(TrackState &track, float z_rad_m,
                                    bool has_speed, float v_rad_mps) {
  if (!track.initialized) {
    return;
  }

  if (!has_speed) {
    const int idx = 0;
    const float y = z_rad_m - track.x[idx];
    const float s =
        std::max(track.P[idx * 4 + idx] + config_.ekf_r_radar_z, 1e-5f);

    float K[4] = {0.0f};
    for (int r = 0; r < 4; ++r) {
      K[r] = track.P[r * 4 + idx] / s;
      track.x[r] += K[r] * y;
    }

    float P_old[16];
    for (int i = 0; i < 16; ++i) {
      P_old[i] = track.P[i];
    }

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        track.P[r * 4 + c] = P_old[r * 4 + c] - K[r] * P_old[idx * 4 + c];
      }
    }

    return;
  }

  // 2D radar update on [z, vz].
  const float y0 = z_rad_m - track.x[0];
  const float y1 = v_rad_mps - track.x[1];

  const float s00 = track.P[0] + config_.ekf_r_radar_z;
  const float s01 = track.P[1];
  const float s10 = track.P[4];
  const float s11 = track.P[5] + config_.ekf_r_radar_vz;

  float det = s00 * s11 - s01 * s10;
  if (std::abs(det) < 1e-6f) {
    det = (det >= 0.0f) ? 1e-6f : -1e-6f;
  }

  const float inv00 = s11 / det;
  const float inv01 = -s01 / det;
  const float inv10 = -s10 / det;
  const float inv11 = s00 / det;

  float K0[4] = {0.0f};
  float K1[4] = {0.0f};
  for (int r = 0; r < 4; ++r) {
    const float p_r0 = track.P[r * 4 + 0];
    const float p_r1 = track.P[r * 4 + 1];
    K0[r] = p_r0 * inv00 + p_r1 * inv10;
    K1[r] = p_r0 * inv01 + p_r1 * inv11;
  }

  for (int r = 0; r < 4; ++r) {
    track.x[r] += K0[r] * y0 + K1[r] * y1;
  }
  track.x[2] = normalizeAngle(track.x[2]);

  float P_old[16];
  for (int i = 0; i < 16; ++i) {
    P_old[i] = track.P[i];
  }

  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      track.P[r * 4 + c] = P_old[r * 4 + c] - K0[r] * P_old[0 * 4 + c] -
                           K1[r] * P_old[1 * 4 + c];
    }
  }
}

void SensorFusion::maybeCleanupTracks(uint64_t now_ns) {
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    const TrackState &track = it->second;
    const uint64_t cam_age_ms =
        (track.last_camera_ns > 0 && now_ns >= track.last_camera_ns)
            ? (now_ns - track.last_camera_ns) / 1000000ULL
            : std::numeric_limits<uint64_t>::max();
    const uint64_t radar_age_ms =
        (track.last_radar_ns > 0 && now_ns >= track.last_radar_ns)
            ? (now_ns - track.last_radar_ns) / 1000000ULL
            : std::numeric_limits<uint64_t>::max();

    if (cam_age_ms > config_.track_cleanup_ms &&
        radar_age_ms > config_.track_cleanup_ms) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

float SensorFusion::calculateIOU(const cv::Rect2f &a,
                                 const cv::Rect2f &b) const {
  const float x_left = std::max(a.x, b.x);
  const float y_top = std::max(a.y, b.y);
  const float x_right = std::min(a.x + a.width, b.x + b.width);
  const float y_bottom = std::min(a.y + a.height, b.y + b.height);

  if (x_right < x_left || y_bottom < y_top) {
    return 0.0f;
  }

  const float intersection = (x_right - x_left) * (y_bottom - y_top);
  const float a_area = a.width * a.height;
  const float b_area = b.width * b.height;

  return intersection / std::max(a_area + b_area - intersection, 1e-6f);
}

float SensorFusion::estimateDistance(float v_bottom, float fy_scaled,
                                     float cy_scaled, float pitch_rad,
                                     float cam_height_m) const {
  const float alpha = std::atan((v_bottom - cy_scaled) / fy_scaled);
  const float angle = pitch_rad + alpha;

  if (angle <= 0.001f) {
    return 150.0f;
  }

  return cam_height_m / std::tan(angle);
}

bool SensorFusion::inRadarROI(float u, float /*v*/,
                              const cv::Rect2f &roi) const {
  return (u >= roi.x && u <= (roi.x + roi.width));
}

float SensorFusion::computeTTC(float z_rad, float v_rad_approaching) const {
  if (v_rad_approaching > 0.0f) {
    return z_rad / v_rad_approaching;
  }
  return std::numeric_limits<float>::infinity();
}

} // namespace adas
