// File: include/adas/stage_e/BEVDashboard.hpp
// Dedicated Bird's-Eye View dashboard for Stage E raw+fused debug rendering.
#pragma once

#include "adas/stage_a/BSDReceiver.hpp"
#include "adas/stage_e/FCWMonitor.hpp"
#include "adas/stage_e/SensorFusion.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace adas {

struct EgoDebugSnapshot {
  bool valid = false;
  float ego_speed_mps = 0.0f;
  float pitch_rad = 0.0f;
  float roll_rad = 0.0f;
  uint64_t timestamp_ns = 0;
};

/// Full payload for one BEV update tick.
struct BEVInputFrame {
  bool has_camera_batch = false;
  DetBatch camera_batch;
  bool has_radar_targets = false;
  RadarTargets radar_targets;
  std::vector<FusedObject> fused_objects;
  std::optional<uint64_t> fcw_focus_object_id;
  std::optional<FCWAlert> fcw_alert_context;
  std::optional<FCWEvaluation> fcw_eval_context;
  std::optional<FCWDebugSnapshot> fcw_debug_context;
  std::optional<EgoDebugSnapshot> ego_debug_context;
  uint64_t now_ns = 0;
};

class BEVDashboard {
public:
  /// Constructor
  /// @param bsd_receiver Pointer to BSD receiver for presence-state polygons
  /// @param c_x Principal point X (camera intrinsics)
  /// @param f_x Horizontal focal length (camera intrinsics)
  /// @param dead_track_cleanup_ms Remove tracks after this long with no cam/range
  /// @param ttc_hold_ms Hold highlighted FCW-trigger object for this duration
  BEVDashboard(BSDReceiver *bsd_receiver, float c_x, float f_x,
               uint32_t dead_track_cleanup_ms = 500,
               uint32_t ttc_hold_ms = 5000);
  ~BEVDashboard();

  // Non-copyable
  BEVDashboard(const BEVDashboard &) = delete;
  BEVDashboard &operator=(const BEVDashboard &) = delete;

  /// Starts the dedicated dashboard rendering thread
  void start();

  /// Stops the dashboard and closes the window
  void stop();

  /// Safely updates the data payload used by the rendering thread
  void update(const BEVInputFrame &frame);

  /// Hot-update the display track cleanup TTL to match fusion retention.
  void setDeadTrackCleanupMs(uint32_t dead_track_cleanup_ms) {
    dead_track_cleanup_ms_.store(dead_track_cleanup_ms,
                                 std::memory_order_relaxed);
  }

private:
  struct Track {
    uint64_t object_id = UINT64_MAX;
    float corridor_angle_rad = 0.0f;
    float x_offset_m = 0.0f;
    float z_m = 0.0f;
    float radial_vel_mps = 0.0f;
    bool has_cam_est_range = false;
    float cam_est_range_m = 0.0f;
    bool has_radar = false;
    float ttc_s = -1.0f;
    float fusion_quality = 0.0f;
    float dz_cam_radar_m = -1.0f;
    uint32_t camera_age_ms = 0;
    uint32_t radar_age_ms = 0;
    bool is_predicted_camera = false;
    bool is_aggressive_mode = false;
    bool speed_fresh = false;
    uint32_t speed_age_ms = 0;
    VelocitySource velocity_source = VelocitySource::None;
    uint64_t last_cam_update_ns = 0;
    uint64_t last_range_update_ns = 0;
    uint64_t ttc_hold_until_ns = 0;
    bool has_fcw_trigger_speed = false;
    float fcw_trigger_speed_mps = 0.0f;
    uint64_t fcw_eval_until_ns = 0;
    uint8_t fcw_eval_level = 0;
    float fcw_eval_risk = 0.0f;
    bool fcw_eval_used_camera_drop_grace = false;
    bool has_crosshair = false;
    std::string label;
  };

  void renderLoop();
  void applyFrameUpdate(const BEVInputFrame &frame);

  BSDReceiver *bsd_receiver_;
  float c_x_;
  float f_x_;
  std::atomic<uint32_t> dead_track_cleanup_ms_;
  uint32_t ttc_hold_ms_;

  BEVInputFrame latest_frame_;
  uint64_t latest_frame_seq_ = 0;
  std::mutex data_mutex_;
  std::unordered_map<uint64_t, Track> tracks_;
  std::optional<FCWDebugSnapshot> latched_fcw_debug_snapshot_;
  uint64_t latched_fcw_debug_until_ns_ = 0;
  uint64_t latched_fcw_debug_last_update_ns_ = 0;

  std::thread thread_;
  std::atomic<bool> running_{false};

  struct SourceSnapshot {
    int count = 0;
    uint64_t timestamp_ns = 0;
    bool healthy = false;
  };

  SourceSnapshot latest_camera_source_;
  SourceSnapshot latest_radar_source_;
  std::optional<EgoDebugSnapshot> latest_ego_debug_snapshot_;
};

} // namespace adas
