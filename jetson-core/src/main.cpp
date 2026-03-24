// File: src/main.cpp
// ADAS Pipeline Entry Point - Interactive CLI with Stage A ingest and Stage E
// fusion/alerting
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
// POSIX process management (fork/exec/kill) for deepstream-app
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "adas/common/Clock.hpp"
#include "adas/common/Config.hpp"
#include "adas/common/Globals.hpp"
#include "adas/common/MetricsLogger.hpp"
#include "adas/main_brain/AlertGenerator.hpp"
#include "adas/main_brain/BleFragmenter.hpp"
#include "adas/main_brain/FCWAlertAdapter.hpp"
#include "adas/main_brain/SimpleBleServer.hpp"
#include "adas/recording/Recorder.hpp"
#include "adas/stage_a/DeviceWizard.hpp"
#include "adas/stage_a/IngestManager.hpp"
// Front camera detections are provided by external DeepStream IPC.

#include "adas/stage_a/BSDReceiver.hpp"
#include "adas/stage_e/BEVDashboard.hpp"
#include "adas/stage_e/EgoFrame.hpp"
#include "adas/stage_e/FCWMonitor.hpp"
#include "adas/stage_e/SensorFusion.hpp"

namespace {

// Global managers
std::unique_ptr<adas::IngestManager> g_ingest_manager;
std::atomic<bool> g_shutdown_requested{false};
std::atomic<bool> g_pipeline_running{false};
std::atomic<bool> g_rtsp_streaming{false};

// DeepStream C binary subprocess (deepstream-app)
pid_t g_ds_pid = -1;

void launchDeepStreamApp() {
  pid_t pid = fork();
  if (pid == 0) {
    // Child: exec the C application and never return
    if (chdir("/home/capstone-66/dashcamnet") != 0) {
      std::cerr << "[DS] Failed to change directory to "
                   "/home/capstone-66/dashcamnet\n";
      _exit(1);
    }

    std::vector<const char *> args = {
        "/opt/nvidia/deepstream/deepstream-6.0/sources/apps/sample_apps/"
        "deepstream-app/deepstream-app",
        "-c", "deepstream_app.txt", nullptr};

    execv(args[0], const_cast<char *const *>(args.data()));
    std::cerr << "[DS] exec deepstream-app failed\n";
    _exit(1);
  } else if (pid > 0) {
    g_ds_pid = pid;
    std::cout << "[DS] deepstream-app launched (PID " << pid << ")\n";
  } else {
    std::cerr << "[DS] fork() failed\n";
  }
}

void stopDeepStreamApp() {
  if (g_ds_pid > 0) {
    std::cout << "[DS] Stopping deepstream-app (PID " << g_ds_pid << ")\n";
    ::kill(g_ds_pid, SIGTERM);
    // Give it up to 3 s to shut down gracefully, then SIGKILL
    for (int i = 0; i < 30; ++i) {
      int wstatus;
      pid_t r = waitpid(g_ds_pid, &wstatus, WNOHANG);
      if (r > 0) {
        g_ds_pid = -1;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ::kill(g_ds_pid, SIGKILL);
    waitpid(g_ds_pid, nullptr, 0);
    g_ds_pid = -1;
    std::cout << "[DS] deepstream-app stopped\n";
  }
}

// Status bar thread
std::thread g_status_thread;
std::atomic<bool> g_status_running{false};
std::chrono::steady_clock::time_point g_pipeline_start_time;

// Visualization thread
std::thread g_visualizer_thread;
std::atomic<bool> g_visualizer_running{false};

// Stage E: Fusion + FCW + EgoFrame
std::unique_ptr<adas::SensorFusion> g_sensor_fusion;
std::unique_ptr<adas::FCWMonitor> g_fcw_monitor;
std::unique_ptr<adas::EgoFrame> g_ego_frame;
std::atomic<bool> g_fcw_alert_active{false};
std::atomic<int> g_fcw_ttc_ms{
    0}; // TTC in milliseconds (avoids float atomic availability issues)
std::atomic<float> g_latest_phone_gps_speed_mps{0.0f};
std::atomic<uint64_t> g_latest_phone_gps_rx_ns{0};

// Stage A: BSD Receiver
std::unique_ptr<adas::BSDReceiver> g_bsd_receiver;

// Stage E: BEV Dashboard
std::unique_ptr<adas::BEVDashboard> g_bev_dashboard;

// BLE Server for mobile app communication
std::unique_ptr<adas::SimpleBleServer> g_ble_server;

// Metrics logger for validation
std::unique_ptr<adas::MetricsLogger> g_metrics_logger;

// Recording and replay
std::unique_ptr<adas::Recorder> g_recorder;
std::string g_record_dir = "./recordings";
std::string g_replay_file;
float g_replay_speed = 1.0f;
bool g_replay_fast = false;
int g_stage_e_sensitivity_level = 3; // 1=least sensitive, 5=most sensitive
float g_fcw_min_trigger_object_speed_gate_mps =
    0.0f; // 0 means no final trigger speed gate
std::atomic<float> g_active_radar_tx_offset_m{0.0127f};

class RangeDistanceCaptureLogger {
public:
  bool start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled_.load(std::memory_order_relaxed)) {
      return true;
    }

    const std::string output_dir = defaultOutputDir();
    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);

    file_path_ =
        output_dir + "/cam_radar_range_capture_" + timestampSuffix() + ".csv";
    file_.open(file_path_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
      file_path_.clear();
      return false;
    }

    file_ << "capture_ts_ns,object_id,object_class,score,fusion_quality,"
             "theta_rad,camera_range_m,camera_slant_m,radar_range_adj_m,"
             "radar_range_raw_m,range_error_m,v_bottom_px,box_x,box_y,box_w,"
             "box_h,centroid_x,centroid_y,pitch_rad,cam_height_m,"
             "camera_age_ms,radar_age_ms,speed_fresh,speed_age_ms,"
             "radial_vel_mps,is_predicted_camera,is_aggressive_mode\n";
    file_.flush();
    sample_count_.store(0, std::memory_order_relaxed);
    enabled_.store(true, std::memory_order_release);
    return true;
  }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_.store(false, std::memory_order_release);
    if (file_.is_open()) {
      file_.flush();
      file_.close();
    }
  }

  bool isEnabled() const { return enabled_.load(std::memory_order_acquire); }

  uint64_t getSampleCount() const {
    return sample_count_.load(std::memory_order_relaxed);
  }

  std::string getFilePath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_path_;
  }

  void logFusedObjects(const std::vector<adas::FusedObject> &fused,
                       float pitch_rad, float cam_height_m, float radar_tx_m,
                       uint64_t capture_ts_ns) {
    if (!enabled_.load(std::memory_order_acquire) || fused.empty()) {
      return;
    }

    std::ostringstream rows;
    rows << std::fixed << std::setprecision(6);
    uint64_t accepted = 0;

    for (const auto &obj : fused) {
      if (!shouldCapture(obj)) {
        continue;
      }

      const float v_bottom_px = obj.box_px.y + obj.box_px.height;
      const float cos_theta = std::cos(obj.theta_rad);
      const float camera_slant_m =
          (std::abs(cos_theta) > 1e-4f) ? (obj.z_cam_m / cos_theta) : 0.0f;
      const float radar_range_raw_m = obj.range_m + radar_tx_m;
      const float range_error_m = obj.z_cam_m - obj.range_m;

      rows << capture_ts_ns << ',' << obj.object_id << ','
           << adas::objectClassToString(obj.object_class) << ',' << obj.score
           << ',' << obj.fusion_quality << ',' << obj.theta_rad << ','
           << obj.z_cam_m << ',' << camera_slant_m << ',' << obj.range_m << ','
           << radar_range_raw_m << ',' << range_error_m << ',' << v_bottom_px
           << ',' << obj.box_px.x << ',' << obj.box_px.y << ','
           << obj.box_px.width << ',' << obj.box_px.height << ','
           << obj.centroid_px.x << ',' << obj.centroid_px.y << ',' << pitch_rad
           << ',' << cam_height_m << ',' << obj.camera_age_ms << ','
           << obj.radar_age_ms << ',' << (obj.speed_fresh ? 1 : 0) << ','
           << obj.speed_age_ms << ',' << obj.radial_vel_mps << ','
           << (obj.is_predicted_camera ? 1 : 0) << ','
           << (obj.is_aggressive_mode ? 1 : 0) << '\n';
      ++accepted;
    }

    if (accepted == 0) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_.load(std::memory_order_relaxed) || !file_.is_open()) {
      return;
    }
    file_ << rows.str();
    sample_count_.fetch_add(accepted, std::memory_order_relaxed);
  }

private:
  static bool shouldCapture(const adas::FusedObject &obj) {
    constexpr float kMinFusionQuality = 0.60f;
    constexpr float kMaxAbsThetaRad = 10.0f * 3.14159265358979323846f / 180.0f;
    constexpr uint32_t kMaxCameraAgeMs = 150;
    constexpr uint32_t kMaxRadarAgeMs = 150;

    if (obj.object_id == UINT64_MAX || !obj.has_radar || obj.z_cam_m <= 0.1f ||
        obj.range_m <= 0.1f) {
      return false;
    }
    if (obj.is_predicted_camera || obj.fusion_quality < kMinFusionQuality) {
      return false;
    }
    if (obj.camera_age_ms > kMaxCameraAgeMs ||
        obj.radar_age_ms > kMaxRadarAgeMs) {
      return false;
    }
    if (std::abs(obj.theta_rad) > kMaxAbsThetaRad) {
      return false;
    }

    switch (static_cast<adas::ObjectClass>(obj.object_class)) {
    case adas::ObjectClass::Car:
    case adas::ObjectClass::Bicycle:
    case adas::ObjectClass::Person:
      return true;
    default:
      return false;
    }
  }

  static std::string defaultOutputDir() {
    const char *home = std::getenv("HOME");
    if (home && *home) {
      return std::string(home) + "/Desktop";
    }
    return ".";
  }

  static std::string timestampSuffix() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
#ifdef _WIN32
    localtime_s(&tm_now, &time_t_now);
#else
    localtime_r(&time_t_now, &tm_now);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  mutable std::mutex mutex_;
  std::ofstream file_;
  std::string file_path_;
  std::atomic<bool> enabled_{false};
  std::atomic<uint64_t> sample_count_{0};
};

RangeDistanceCaptureLogger g_range_distance_capture_logger;

std::string formatUptime(std::chrono::seconds uptime) {
  int hours = uptime.count() / 3600;
  int minutes = (uptime.count() % 3600) / 60;
  int seconds = uptime.count() % 60;

  std::ostringstream ss;
  if (hours > 0) {
    ss << hours << "h " << minutes << "m";
  } else if (minutes > 0) {
    ss << minutes << "m " << seconds << "s";
  } else {
    ss << seconds << "s";
  }
  return ss.str();
}

int clampSensitivityLevel(int level) { return std::max(1, std::min(5, level)); }

std::string normalizeRadarOutputMode(std::string mode) {
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (mode == "combined_native") {
    return "combined_native";
  }
  return "split_range";
}

const char *radarOutputModeLabel(const std::string &mode) {
  if (normalizeRadarOutputMode(mode) == "combined_native") {
    return "COMBINED_NATIVE";
  }
  return "SPLIT_RANGE";
}

float sensitivityFactor(int level) {
  // Levels 1..5 map to 0.85, 0.95, 1.00, 1.05, 1.15 (tighter around L3).
  switch (clampSensitivityLevel(level)) {
  case 1:
    return 0.85f;
  case 2:
    return 0.95f;
  case 3:
    return 1.00f;
  case 4:
    return 1.05f;
  case 5:
    return 1.15f;
  default:
    return 1.00f;
  }
}

const char *sensitivityLabel(int level) {
  switch (clampSensitivityLevel(level)) {
  case 1:
    return "LOW";
  case 2:
    return "MED-LOW";
  case 3:
    return "MEDIUM";
  case 4:
    return "MED-HIGH";
  case 5:
    return "HIGH";
  default:
    return "MEDIUM";
  }
}

uint32_t scaleU32(uint32_t value, float factor, uint32_t min_v,
                  uint32_t max_v) {
  const float scaled = std::round(static_cast<float>(value) * factor);
  const float clamped =
      std::clamp(scaled, static_cast<float>(min_v), static_cast<float>(max_v));
  return static_cast<uint32_t>(clamped);
}

void applyConfiguredFusionHoldTimings(
    adas::FusionConfig &fusion_config, adas::FCWMonitor::Config &fcw_config,
    const adas::StageEFusionConfig &stage_e_config) {
  fusion_config.camera_hold_ms =
      static_cast<uint32_t>(std::max(0, stage_e_config.camera_hold_ms));
  fusion_config.radar_hold_ms =
      static_cast<uint32_t>(std::max(0, stage_e_config.radar_hold_ms));
  fusion_config.track_cleanup_ms =
      static_cast<uint32_t>(std::max(0, stage_e_config.track_cleanup_ms));
  fusion_config.predicted_camera_threshold_ms = static_cast<uint32_t>(
      std::max(0, stage_e_config.predicted_camera_threshold_ms));
  fcw_config.camera_hold_ms = fusion_config.camera_hold_ms;
}

bool validateFusionHoldTimings(int camera_hold_ms, int radar_hold_ms,
                               int track_cleanup_ms,
                               int predicted_camera_threshold_ms,
                               std::string &error) {
  if (camera_hold_ms < 50 || radar_hold_ms < 50 || track_cleanup_ms < 50 ||
      predicted_camera_threshold_ms < 0) {
    error = "All hold timings must be >= 50 ms except predicted threshold, "
            "which must be >= 0.";
    return false;
  }
  if (track_cleanup_ms < std::max(camera_hold_ms, radar_hold_ms)) {
    error = "Track cleanup must be >= both camera hold and radar hold so "
            "tracks do not disappear early.";
    return false;
  }
  if (predicted_camera_threshold_ms > track_cleanup_ms) {
    error = "Predicted-camera threshold must be <= track cleanup.";
    return false;
  }
  return true;
}

void applyStageESensitivity(adas::FusionConfig &fusion_config,
                            adas::FCWMonitor::Config &fcw_config, int level) {
  const int clamped_level = clampSensitivityLevel(level);
  const float sens = sensitivityFactor(clamped_level);
  const float inv_sens = std::max(0.5f, 1.0f / sens);

  // Fusion association sensitivity.
  fusion_config.ttc_aggressive_s =
      std::clamp(fusion_config.ttc_aggressive_s * sens, 2.0f, 6.0f);
  fusion_config.normal_angle_gate_deg =
      std::clamp(fusion_config.normal_angle_gate_deg * sens, 8.0f, 22.0f);
  fusion_config.aggressive_angle_gate_deg =
      std::clamp(fusion_config.aggressive_angle_gate_deg * sens, 10.0f, 30.0f);
  if (fusion_config.aggressive_angle_gate_deg <
      fusion_config.normal_angle_gate_deg) {
    fusion_config.aggressive_angle_gate_deg =
        fusion_config.normal_angle_gate_deg;
  }
  fusion_config.aggressive_range_scale =
      std::clamp(fusion_config.aggressive_range_scale * sens, 1.0f, 2.5f);
  fusion_config.camera_hold_ms =
      scaleU32(fusion_config.camera_hold_ms, sens, 200, 1400);

  // FCW risk/escalation sensitivity.
  fcw_config.ttc_threshold_s =
      std::clamp(fcw_config.ttc_threshold_s * sens, 2.0f, 5.0f);
  fcw_config.ttc_immediate_warn_s =
      std::clamp(fcw_config.ttc_immediate_warn_s * sens, 1.8f, 4.2f);
  fcw_config.ttc_immediate_critical_s =
      std::clamp(fcw_config.ttc_immediate_critical_s * sens, 0.7f, 2.0f);
  fcw_config.min_closing_speed_mps =
      std::clamp(fcw_config.min_closing_speed_mps * inv_sens, 0.15f, 1.2f);
  fcw_config.min_fusion_quality =
      std::clamp(fcw_config.min_fusion_quality * inv_sens, 0.08f, 0.50f);
  fcw_config.caution_risk_threshold =
      std::clamp(fcw_config.caution_risk_threshold * inv_sens, 0.20f, 0.80f);
  fcw_config.warn_risk_threshold =
      std::clamp(fcw_config.warn_risk_threshold * inv_sens, 0.30f, 0.90f);
  fcw_config.critical_risk_threshold =
      std::clamp(fcw_config.critical_risk_threshold * inv_sens, 0.40f, 0.98f);
  fcw_config.path_half_width_m =
      std::clamp(fcw_config.path_half_width_m * sens, 0.6f, 1.8f);
  fcw_config.path_width_growth_per_m =
      std::clamp(fcw_config.path_width_growth_per_m * sens, 0.015f, 0.09f);

  // Timing/hysteresis. Higher sensitivity => escalate faster, hold longer.
  fcw_config.caution_dwell_ms =
      scaleU32(fcw_config.caution_dwell_ms, inv_sens, 20, 800);
  fcw_config.warn_dwell_ms =
      scaleU32(fcw_config.warn_dwell_ms, inv_sens, 20, 800);
  fcw_config.critical_dwell_ms =
      scaleU32(fcw_config.critical_dwell_ms, inv_sens, 10, 600);
  fcw_config.clear_dwell_ms =
      scaleU32(fcw_config.clear_dwell_ms, sens, 20, 1200);
  fcw_config.camera_drop_track_hold_ms =
      scaleU32(fcw_config.camera_drop_track_hold_ms, sens, 400, 3000);
  fcw_config.camera_drop_radar_recent_ms =
      scaleU32(fcw_config.camera_drop_radar_recent_ms, sens, 50, 800);
  fcw_config.invalid_demote_grace_ms =
      scaleU32(fcw_config.invalid_demote_grace_ms, sens, 60, 1200);
  fcw_config.invalid_state_hold_ms =
      scaleU32(fcw_config.invalid_state_hold_ms, sens, 80, 1600);

  // Keep FCW camera hold in lockstep with fusion camera hold.
  fcw_config.camera_hold_ms = fusion_config.camera_hold_ms;
}

// Visualization control flag (set to false for production)
std::atomic<bool> g_visualizer_enabled{true};

adas::Alert buildRcwAlert(const adas::RcwState &state, uint64_t now_ns) {
  adas::Alert alert;
  alert.t_ms = now_ns / 1000000ULL;
  alert.id =
      "rcw_" + std::to_string(alert.t_ms) + "_" + std::to_string(state.h.seq);
  alert.type = adas::AlertType::RCW;
  alert.direction = "rear";
  alert.severity = adas::Severity::Warning;
  alert.ttl_ms = 500;

  std::ostringstream rationale;
  rationale << "{\"alert\":" << static_cast<int>(state.alert)
            << ",\"status\":" << static_cast<int>(state.status) << "}";
  alert.rationale = rationale.str();

  alert.object_id = std::nullopt;
  alert.sources = {"RearRCW"};
  alert.schemaVersion = "v1.0";
  alert.confidence = 1.0f;
  return alert;
}

int buildHealthMask() {
  if (!g_ingest_manager) {
    return 0;
  }

  const auto health = g_ingest_manager->getHealth();
  const auto isHealthy = [&](adas::Mount mount, bool default_value) {
    const auto it = health.sensor_health.find(mount);
    return it == health.sensor_health.end() ? default_value : it->second;
  };

  const bool front_cam_ok = isHealthy(adas::Mount::FrontCam, true);
  const bool rear_rcw_ok =
      isHealthy(adas::Mount::IMU, true) &&
      isHealthy(adas::Mount::RearCornerRadarL, true) &&
      isHealthy(adas::Mount::RearCornerRadarR, true);
  const bool front_radar_ok = isHealthy(adas::Mount::FrontRadar, true);

  int mask = 0;
  if (!front_cam_ok) {
    mask |= (1 << 0);
  }
  if (!rear_rcw_ok) {
    mask |= (1 << 1);
  }
  if (!front_radar_ok) {
    mask |= (1 << 2);
  }
  return mask;
}

int buildBsdMask() {
  int mask = 0;
  if (g_bsd_receiver) {
    if (g_bsd_receiver->getLeftBSDState()) {
      mask |= (1 << 0);
    }
    if (g_bsd_receiver->getRightBSDState()) {
      mask |= (1 << 1);
    }
  }
  return mask;
}

int currentBleSpeedKmh() {
  constexpr uint64_t kGpsSpeedFreshNs = 3ULL * 1000ULL * 1000ULL * 1000ULL;

  const uint64_t now_ns = adas::Clock::now_ns();
  const uint64_t gps_rx_ns =
      g_latest_phone_gps_rx_ns.load(std::memory_order_relaxed);
  if (gps_rx_ns == 0 || (now_ns - gps_rx_ns) > kGpsSpeedFreshNs) {
    return 0;
  }

  const float speed_mps =
      g_latest_phone_gps_speed_mps.load(std::memory_order_relaxed);
  return static_cast<int>(std::round(std::fabs(speed_mps) * 3.6f));
}

// Stage E thread: fusion, alerting, BLE, metrics, and (optionally)
// visualization
void visualizationThread() {
  const bool display_enabled = g_visualizer_enabled.load();
  std::cout << "[StageE] Thread started (display "
            << (display_enabled ? "enabled" : "disabled") << ")\n";

  adas::RcwState latest_rcw_state;
  bool have_rcw_state = false;

  while (g_visualizer_running.load() && !g_shutdown_requested.load()) {
    bool got_camera_update = false;
    adas::DetBatch batch;
    if (g_ingest_manager) {
      while (g_ingest_manager->getFrontCamDetQueue().try_pop(batch)) {
        got_camera_update = true;
      }
    }

    if (g_ingest_manager) {
      adas::RcwState rcw_state;
      while (g_ingest_manager->getRcwQueue().try_pop(rcw_state)) {
        latest_rcw_state = rcw_state;
        have_rcw_state = true;
      }
    }

    // DeepStream owns the X11 video window now.
    // Legacy OpenCV cv::namedWindow creation removed.

    adas::RadarTargets radar;
    bool got_radar_update = false;
    if (g_ingest_manager) {
      try {
        auto &radar_queue =
            g_ingest_manager->getRadarQueue(adas::Mount::FrontRadar);
        while (radar_queue.try_pop(radar)) {
          got_radar_update = true;
          if (g_sensor_fusion) {
            const uint64_t radar_now_ns = (radar.h.t_ingest_ns > 0)
                                              ? radar.h.t_ingest_ns
                                              : adas::Clock::now_ns();
            g_sensor_fusion->ingestRadar(radar, radar_now_ns);
          }
        }
      } catch (...) {
        // FrontRadar not configured, radar.targets will be empty
      }
    }

    // Update EgoFrame from IMU data (consumed from ZMQ via IngestManager)
    if (g_ingest_manager && g_ego_frame) {
      auto &imu_queue = g_ingest_manager->getIMUQueue();
      adas::ImuSample imu_sample;
      while (imu_queue.try_pop(imu_sample)) {
        float dt = 0.01f; // Default 100Hz
        if (g_ego_frame->previous_time_ns != 0 &&
            imu_sample.t_capture > g_ego_frame->previous_time_ns) {
          dt = adas::Clock::ns_to_sec(imu_sample.t_capture -
                                      g_ego_frame->previous_time_ns);
        }
        g_ego_frame->previous_time_ns = imu_sample.t_capture;
        g_ego_frame->update(imu_sample, dt);
      }

      if (g_fcw_monitor) {
        g_fcw_monitor->setEgoVelocity(g_ego_frame->getForwardVelocity_mps());
      }
    }

    if (g_sensor_fusion) {
      if (g_ingest_manager) {
        g_sensor_fusion->setPitch(g_ingest_manager->getLatestPitch());
      }
      if (got_camera_update) {
        const uint64_t cam_now_ns = (batch.h.t_ingest_ns > 0)
                                        ? batch.h.t_ingest_ns
                                        : adas::Clock::now_ns();
        g_sensor_fusion->ingestCamera(batch, cam_now_ns);
      }
    }

    const bool have_sensor_tick = got_camera_update || got_radar_update;
    std::vector<adas::FusedObject> fused;
    if (g_sensor_fusion && have_sensor_tick) {
      fused = g_sensor_fusion->getFusedObjects(adas::Clock::now_ns());
      if (g_range_distance_capture_logger.isEnabled()) {
        g_range_distance_capture_logger.logFusedObjects(
            fused, g_sensor_fusion->getPitch(),
            g_sensor_fusion->getCameraHeight(),
            g_active_radar_tx_offset_m.load(std::memory_order_relaxed),
            adas::Clock::now_ns());
      }
    }

    std::optional<adas::FCWAlert> fcw_alert;
    std::optional<adas::FCWEvaluation> fcw_eval;
    std::optional<adas::FCWDebugSnapshot> fcw_debug;
    if (g_fcw_monitor && have_sensor_tick) {
      const uint64_t fcw_now_ns = adas::Clock::now_ns();
      fcw_alert = g_fcw_monitor->check(fused, fcw_now_ns);
      const auto eval = g_fcw_monitor->getLastEvaluation();
      if (eval.has_candidate) {
        fcw_eval = eval;
      }
      const auto debug = g_fcw_monitor->getLastDebugSnapshot();
      fcw_debug = debug;
    }

    g_fcw_alert_active.store(fcw_alert.has_value());

    // BLE Transmission: Heartbeat (1Hz) + Alerts (Immediate)
    if (g_ble_server && g_ble_server->isConnected()) {
      static auto last_ble_send = std::chrono::steady_clock::time_point();

      const bool rcw_active =
          have_rcw_state && static_cast<int>(latest_rcw_state.alert) != 0;
      const bool is_alerting = fcw_alert.has_value() || rcw_active;
      const auto now_time = std::chrono::steady_clock::now();
      const auto time_since =
          std::chrono::duration_cast<std::chrono::milliseconds>(now_time -
                                                                last_ble_send);

      if (is_alerting || time_since.count() > 1000) {
        std::vector<adas::Alert> alerts_to_send;

        const int speed_kmh = currentBleSpeedKmh();
        const int health_mask = buildHealthMask();
        const int bsd_mask = buildBsdMask();

        if (fcw_alert.has_value()) {
          auto alert =
              adas::FCWAlertAdapter::convert(*fcw_alert, adas::Clock::now_ns());
          alerts_to_send.push_back(alert);
        }

        if (rcw_active) {
          alerts_to_send.push_back(
              buildRcwAlert(latest_rcw_state, adas::Clock::now_ns()));
        }

        uint16_t tickId =
            static_cast<uint16_t>(adas::Clock::now_ns() / 50'000'000);

        auto payload = adas::encodeTickPayloadToCbor(
            tickId, speed_kmh, health_mask, bsd_mask, alerts_to_send);

        auto frames = adas::fragmentPayload(tickId, payload, 185);
        for (const auto &frame : frames) {
          g_ble_server->notifyAlertStream(frame);
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        last_ble_send = now_time;
      }
    }

    // Log metrics when camera frame data advanced Stage E.
    if (got_camera_update && g_metrics_logger &&
        g_metrics_logger->isEnabled()) {
      const uint64_t now_ns = adas::Clock::now_ns();
      const double e2e_latency_ms = (now_ns - batch.h.t_ingest_ns) / 1e6;

      const float ttc = fcw_alert.has_value() ? fcw_alert->ttc_s : -1.0f;
      const float range = fcw_alert.has_value() ? fcw_alert->range_m : -1.0f;
      const bool triggered = fcw_alert.has_value();

      g_metrics_logger->logFrame(now_ns / 1e6, // timestamp_ms
                                 batch.h.seq,
                                 batch.inference_time_us / 1000.0, // ms
                                 ttc, range, triggered, e2e_latency_ms);
    }

    if (g_bev_dashboard && have_sensor_tick) {
      adas::BEVInputFrame bev_frame;
      if (got_camera_update) {
        bev_frame.camera_batch = batch;
      }
      if (got_radar_update) {
        bev_frame.radar_targets = radar;
      }
      bev_frame.fused_objects = fused;
      if (fcw_eval.has_value()) {
        bev_frame.fcw_eval_context = *fcw_eval;
      }
      if (fcw_debug.has_value()) {
        bev_frame.fcw_debug_context = *fcw_debug;
      }
      if (fcw_alert.has_value()) {
        bev_frame.fcw_alert_context = *fcw_alert;
        if (fcw_alert->object_id != UINT64_MAX) {
          bev_frame.fcw_focus_object_id = fcw_alert->object_id;
        }
      }
      bev_frame.now_ns = adas::Clock::now_ns();
      g_bev_dashboard->update(bev_frame);
    }

    if (!have_sensor_tick) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::cout << "[StageE] Thread stopped\n";
}
void statusBarThread() {
  while (g_status_running.load() && !g_shutdown_requested.load()) {
    if (!g_ingest_manager) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    auto health = g_ingest_manager->getHealth();

    // Get individual sensor health
    bool front_cam_ok = false;
    bool front_radar_ok = false;
    bool imu_ok = false;

    auto &sh = health.sensor_health;
    if (sh.find(adas::Mount::FrontCam) != sh.end()) {
      front_cam_ok = sh[adas::Mount::FrontCam];
    }
    if (sh.find(adas::Mount::FrontRadar) != sh.end()) {
      front_radar_ok = sh[adas::Mount::FrontRadar];
    }
    if (sh.find(adas::Mount::IMU) != sh.end()) {
      imu_ok = sh[adas::Mount::IMU];
    }

    // Calculate uptime
    auto now = std::chrono::steady_clock::now();
    auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
        now - g_pipeline_start_time);

    // Format status bar
    std::cout << "\r[ADAS] " << "FrontCam:"
              << (front_cam_ok ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m") << " "
              << "FrontRadar:"
              << (front_radar_ok ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m")
              << " "
              << "IMU:" << (imu_ok ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m")
              << " | " << "Drops:" << health.total_drops << " | "
              << formatUptime(uptime) << "     " << std::flush;

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::cout << "\n"; // Clean line after stopping
}

void signalHandler(int signum) {
  std::cout << "\n[Main] Received signal " << signum
            << ", initiating shutdown...\n";
  g_shutdown_requested.store(true, std::memory_order_relaxed);
}

void printBanner() {
  std::cout << R"(

    ===========================================================================
                                                                       
         AAAAA  DDDD    AAAAA  SSSSS      PPPP   III  PPPP   EEEEE     
        AA   AA DD  DD AA   AA SS        PP  PP  III PP  PP EE         
        AAAAAAA DD   DD AAAAAAA SSSSS    PPPPPP  III PPPPPP EEEEE      
        AA   AA DD  DD AA   AA     SS    PP      III PP     EE         
        AA   AA DDDD   AA   AA SSSSS     PP      III PP     EEEEE      
                                                                       
                    ADAS Pipeline - Interactive Mode                   
                                                                       
    ===========================================================================
    )" << std::endl;

  // Pause for 2 seconds so users can see the banner
  std::this_thread::sleep_for(std::chrono::seconds(2));
}

std::vector<std::string> enumerateActiveUsbCameraNodes() {
  std::vector<std::string> active_devices;
  const auto devices = adas::DeviceWizard::enumerateVideoDevices();

  for (const auto &device : devices) {
    if (adas::DeviceWizard::testVideoDevice(device)) {
      active_devices.push_back(device);
    }
  }

  return active_devices;
}

void runUbuntuCameraNodeSwapHotfix() {
#ifndef __linux__
  std::cout << "[Swap] Ubuntu camera node swap is only supported on Linux.\n";
  return;
#else
  if (g_pipeline_running.load()) {
    std::cout << "[Swap] Please stop the pipeline first.\n";
    return;
  }

  const auto active_devices = enumerateActiveUsbCameraNodes();
  if (active_devices.size() < 2) {
    std::cout << "[Swap] Need at least 2 active USB camera nodes. Found "
              << active_devices.size() << ".\n";
    return;
  }

  std::cout << "\n[Swap] Active USB camera nodes (camera mapper method):\n";
  for (size_t i = 0; i < active_devices.size(); ++i) {
    std::cout << "  " << (i + 1) << ") " << active_devices[i] << "\n";
  }

  int choice_a = 0;
  int choice_b = 0;
  std::cout << "  Select first node to swap [1-" << active_devices.size()
            << "]: ";
  std::cin >> choice_a;
  if (std::cin.fail() || choice_a < 1 ||
      choice_a > static_cast<int>(active_devices.size())) {
    std::cin.clear();
    std::cin.ignore(10000, '\n');
    std::cout << "[Swap] Invalid first selection.\n";
    return;
  }

  std::cout << "  Select second node to swap [1-" << active_devices.size()
            << "]: ";
  std::cin >> choice_b;
  if (std::cin.fail() || choice_b < 1 ||
      choice_b > static_cast<int>(active_devices.size())) {
    std::cin.clear();
    std::cin.ignore(10000, '\n');
    std::cout << "[Swap] Invalid second selection.\n";
    return;
  }
  std::cin.ignore(10000, '\n');

  if (choice_a == choice_b) {
    std::cout << "[Swap] Select two different nodes.\n";
    return;
  }

  const std::string dev_a = active_devices[choice_a - 1];
  const std::string dev_b = active_devices[choice_b - 1];
  const std::string tmp_dev = "/dev/video_temp_swap";

  if (::access(tmp_dev.c_str(), F_OK) == 0) {
    std::cout << "[Swap] Temp node already exists at " << tmp_dev
              << ". Remove it first, then retry.\n";
    return;
  }

  std::cout << "\n[Swap] Selected nodes:\n";
  std::cout << "  A: " << dev_a << "\n";
  std::cout << "  B: " << dev_b << "\n";
  std::cout << "  Temp: " << tmp_dev << "\n";
  std::cout << "  Continue with sudo node swap? (y/N): ";

  std::string confirm;
  std::getline(std::cin, confirm);
  if (confirm != "y" && confirm != "Y") {
    std::cout << "[Swap] Cancelled.\n";
    return;
  }

  const std::string command =
      "sudo mv \"" + dev_a + "\" \"" + tmp_dev + "\" && sudo mv \"" + dev_b +
      "\" \"" + dev_a + "\" && sudo mv \"" + tmp_dev + "\" \"" + dev_b + "\"";

  std::cout << "[Swap] Executing: " << command << "\n";
  const int ret = std::system(command.c_str());
  if (ret != 0) {
    std::cout << "[Swap] Node swap command failed (code " << ret
              << "). Check sudo permissions and node availability.\n";
    return;
  }

  std::cout << "[Swap] Camera nodes swapped successfully:\n";
  std::cout << "  " << dev_a << " <-> " << dev_b << "\n";
#endif
}

void printMenu(const adas::Config &config) {
  std::cout << "\n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "                    MAIN MENU                                 \n";
  std::cout
      << "==============================================================\n";
  std::cout << "  1) Start Pipeline\n";
  std::cout << "  2) Stop Pipeline\n";
  std::cout << "  3) Show Status\n";
  std::cout << "  4) Run Device Wizard (USB cameras)\n";
  std::cout << "  5) Run Camera Calibration\n";
  std::cout << "  6) Register Pi4 Network Devices\n";
  std::cout << "  7) Test RTT to Pi4\n";
  std::cout << "  8) Toggle Verbose Mode ["
            << (adas::g_verbose_mode.load() ? "ON" : "OFF") << "]\n";
  std::cout << "  9) View Undistortion Demo\n";
  std::cout << " 10) Toggle Metrics Logging ["
            << (g_metrics_logger && g_metrics_logger->isEnabled() ? "ON"
                                                                  : "OFF")
            << "]\n";
  std::cout << " 11) Toggle Recording ["
            << (g_recorder && g_recorder->isRecording() ? "REC" : "OFF")
            << "]\n";
  std::cout << " 12) Start Pipeline (Replay Mode)\n";
  std::cout << " 13) Edit Camera Config (opens editor, hot-reload)\n";
  std::cout << " 14) Toggle RTSP Stream ["
            << (g_rtsp_streaming.load() ? "ON" : "OFF") << "]\n";
  std::ostringstream sens_ss;
  sens_ss << std::fixed << std::setprecision(2)
          << sensitivityFactor(g_stage_e_sensitivity_level);
  std::cout << " 15) Set Stage E Sensitivity [L" << g_stage_e_sensitivity_level
            << " " << sensitivityLabel(g_stage_e_sensitivity_level) << " x"
            << sens_ss.str() << "]\n";
  std::cout << " 16) Set Front Radar Output Mode ["
            << radarOutputModeLabel(config.front_radar.output_mode) << "]\n";
  std::ostringstream speed_gate_ss;
  speed_gate_ss << std::fixed << std::setprecision(2)
                << g_fcw_min_trigger_object_speed_gate_mps;
  std::cout << " 17) Set FCW Min Trigger Object Speed Gate ["
            << speed_gate_ss.str() << " m/s]\n";
  std::cout << " 18) Ubuntu Camera Node Swap Hotfix\n";
  std::cout << " 19) Toggle Cam/Radar Range Capture ["
            << (g_range_distance_capture_logger.isEnabled()
                    ? ("ON " +
                       std::to_string(
                           g_range_distance_capture_logger.getSampleCount()) +
                       " samples")
                    : std::string("OFF"))
            << "]\n";
  std::cout << " 20) Set Fusion Hold Timings [cam="
            << config.stage_e_fusion.camera_hold_ms
            << " radar=" << config.stage_e_fusion.radar_hold_ms
            << " cleanup=" << config.stage_e_fusion.track_cleanup_ms
            << " pred=" << config.stage_e_fusion.predicted_camera_threshold_ms
            << "]\n";
  std::cout << "  h) Set Camera Height On-The-Fly\n";
  std::cout << "  0) Exit\n";
  std::cout
      << "==============================================================\n";
  std::cout << "  Enter choice: ";
}

void startPipeline(const adas::Config &config,
                   const adas::HardwareMap &hw_map) {
  if (g_pipeline_running.load()) {
    std::cout << "[Main] Pipeline is already running\n";
    return;
  }

  std::cout << "\n[Main] Starting pipeline...\n";

  // Stage A: Ingest
  g_ingest_manager = std::make_unique<adas::IngestManager>(config, hw_map);
  g_ingest_manager->start();

  // ── DeepStream: launch deepstream-app for FrontCam ─────────────────
  // The C application runs nvinfer (DashCamNet) inside GStreamer and shows an
  // nveglglessink window with bounding boxes. It connects to our ZMQ IPC socket
  // to feed detections.
  launchDeepStreamApp();

  // Stage A: BSD Receiver (Pi presence-mode tracking)
  if (g_ingest_manager && !g_ingest_manager->getPiIp().empty()) {
    g_bsd_receiver =
        std::make_unique<adas::BSDReceiver>(g_ingest_manager->getPiIp());
    g_bsd_receiver->start();
  }

  // Stage E: Fusion + FCW + EgoFrame
  adas::FusionConfig fusion_config;
  auto it = config.mounts.find(adas::Mount::FrontCam);
  if (it != config.mounts.end()) {
    fusion_config.cam_height_m = it->second.xyz_m[2];
  }
  fusion_config.radar_half_fov_deg = 15.0f; // 30 degree cone
  fusion_config.dist_gate_m = 5.5f;
  fusion_config.normal_range_gate_m = fusion_config.dist_gate_m;
  fusion_config.ttc_aggressive_s = config.stage_e_fusion.ttc_aggressive_s;
  fusion_config.camera_hold_ms =
      static_cast<uint32_t>(std::max(0, config.stage_e_fusion.camera_hold_ms));
  fusion_config.normal_angle_gate_deg =
      config.stage_e_fusion.normal_angle_gate_deg;
  fusion_config.aggressive_angle_gate_deg =
      config.stage_e_fusion.aggressive_angle_gate_deg;
  fusion_config.aggressive_range_scale =
      config.stage_e_fusion.aggressive_range_scale;
  fusion_config.ekf_q_z = config.stage_e_fusion.ekf_q_z;
  fusion_config.ekf_q_vz = config.stage_e_fusion.ekf_q_vz;
  fusion_config.ekf_q_theta = config.stage_e_fusion.ekf_q_theta;
  fusion_config.ekf_q_theta_dot = config.stage_e_fusion.ekf_q_theta_dot;
  fusion_config.ekf_r_radar_z = config.stage_e_fusion.ekf_r_radar_z;
  fusion_config.ekf_r_radar_vz = config.stage_e_fusion.ekf_r_radar_vz;
  fusion_config.ekf_r_cam_theta = config.stage_e_fusion.ekf_r_cam_theta;
  fusion_config.ekf_r_cam_z_weak = config.stage_e_fusion.ekf_r_cam_z_weak;
  g_active_radar_tx_offset_m.store(fusion_config.radar_tx_m,
                                   std::memory_order_relaxed);

  // Configure FCW with physics-based parameters
  adas::FCWMonitor::Config fcw_config;
  fcw_config.ttc_threshold_s = 3.0f;
  fcw_config.use_physics_fcw = true; // Keep stopping-distance term active.
  fcw_config.friction_coefficient = 0.7f;
  fcw_config.reaction_time_s = 2.5f;
  fcw_config.min_fusion_quality = 0.2f;
  fcw_config.path_half_width_m = 0.9f;
  fcw_config.path_width_growth_per_m = 0.035f;
  fcw_config.warn_risk_threshold = 0.56f;
  fcw_config.critical_risk_threshold = 0.74f;
  fcw_config.ttc_last_ditch_s = 0.85f;
  fcw_config.ttc_immediate_warn_s = 2.8f;
  fcw_config.ttc_immediate_critical_s = 1.2f;
  fcw_config.camera_hold_ms = fusion_config.camera_hold_ms;
  fcw_config.camera_drop_track_hold_ms = 1200;
  fcw_config.camera_drop_radar_recent_ms = 150;
  fcw_config.camera_drop_min_quality = 0.32f;
  fcw_config.invalid_demote_grace_ms = 350;
  fcw_config.invalid_state_hold_ms = 250;
  fcw_config.log_fcw_drop_reasons = true;
  fcw_config.min_trigger_object_speed_mps =
      std::max(0.0f, g_fcw_min_trigger_object_speed_gate_mps);
  applyStageESensitivity(fusion_config, fcw_config,
                         g_stage_e_sensitivity_level);
  applyConfiguredFusionHoldTimings(fusion_config, fcw_config,
                                   config.stage_e_fusion);

  g_sensor_fusion = std::make_unique<adas::SensorFusion>(fusion_config);
  g_fcw_monitor = std::make_unique<adas::FCWMonitor>(fcw_config);

  // Initialize EgoFrame for ego vehicle state from IMU
  g_ego_frame = std::make_unique<adas::EgoFrame>();
  g_ego_frame->init();

  // Initialize BEVDashboard
  g_bev_dashboard = std::make_unique<adas::BEVDashboard>(
      g_bsd_receiver.get(), fusion_config.c_x, fusion_config.f_x,
      fusion_config.track_cleanup_ms);
  g_bev_dashboard->start();

  std::ostringstream sens_start_ss;
  sens_start_ss << std::fixed << std::setprecision(2)
                << sensitivityFactor(g_stage_e_sensitivity_level);
  std::cout << "[Main] Stage E fusion initialized (TTC threshold: "
            << g_fcw_monitor->getThreshold() << "s, Physics FCW: "
            << (fcw_config.use_physics_fcw ? "ENABLED" : "disabled")
            << ", Sensitivity: L" << g_stage_e_sensitivity_level << " ("
            << sensitivityLabel(g_stage_e_sensitivity_level) << ", x"
            << sens_start_ss.str() << "), Min Trigger Speed Gate: "
            << fcw_config.min_trigger_object_speed_mps << " m/s)\n";

  // Initialize BLE Server
  g_ble_server = std::make_unique<adas::SimpleBleServer>();
  if (g_ble_server->initialize()) {
    // Wire GPS callback: phone GPS -> EgoFrame drift correction
    g_ble_server->setOnGpsData([](float speed_mps, uint64_t ts_ms) {
      g_latest_phone_gps_speed_mps.store(speed_mps, std::memory_order_relaxed);
      g_latest_phone_gps_rx_ns.store(adas::Clock::now_ns(),
                                     std::memory_order_relaxed);
      if (g_ego_frame) {
        g_ego_frame->correctWithGpsSpeed(speed_mps);
      }
      // Record GPS data if recording
      if (g_recorder && g_recorder->isRecording()) {
        g_recorder->recordGPS(speed_mps, ts_ms);
      }
    });
    g_ble_server->startAdvertising();
  } else {
    std::cerr << "[Main] WARNING: BLE server failed to initialize\n";
  }

  g_pipeline_running.store(true);
  g_pipeline_start_time = std::chrono::steady_clock::now();

  // Start status bar thread
  g_status_running.store(true);
  g_status_thread = std::thread(statusBarThread);

  // Start visualization thread
  g_visualizer_running.store(true);
  g_visualizer_thread = std::thread(visualizationThread);

  std::cout << "\n[Main] Pipeline started successfully!\n";
  std::cout << "[Main] Press '3' to view status, '2' to stop\n\n";
}

void startReplayPipeline(const std::string &replay_file, float speed,
                         const adas::Config &config,
                         const adas::HardwareMap &hw_map) {
  if (g_pipeline_running.load()) {
    std::cout << "[Main] Pipeline is already running\n";
    return;
  }

  std::cout << "\n[Main] Starting pipeline in REPLAY MODE...\n";

  // Stage A: Ingest (Replay Engine)
  g_ingest_manager = std::make_unique<adas::IngestManager>(config, hw_map);
  if (!g_ingest_manager->initReplay(replay_file, speed)) {
    std::cerr << "[Main] Failed to initialize Replay Mode. Aborting.\n";
    g_ingest_manager.reset();
    return;
  }
  g_ingest_manager->start();

  // Stage E: Fusion + FCW + EgoFrame
  adas::FusionConfig replay_fusion_config;
  replay_fusion_config.radar_half_fov_deg = 15.0f;
  replay_fusion_config.dist_gate_m = 5.5f;
  replay_fusion_config.normal_range_gate_m = replay_fusion_config.dist_gate_m;
  replay_fusion_config.ttc_aggressive_s =
      config.stage_e_fusion.ttc_aggressive_s;
  replay_fusion_config.camera_hold_ms =
      static_cast<uint32_t>(std::max(0, config.stage_e_fusion.camera_hold_ms));
  replay_fusion_config.normal_angle_gate_deg =
      config.stage_e_fusion.normal_angle_gate_deg;
  replay_fusion_config.aggressive_angle_gate_deg =
      config.stage_e_fusion.aggressive_angle_gate_deg;
  replay_fusion_config.aggressive_range_scale =
      config.stage_e_fusion.aggressive_range_scale;
  replay_fusion_config.ekf_q_z = config.stage_e_fusion.ekf_q_z;
  replay_fusion_config.ekf_q_vz = config.stage_e_fusion.ekf_q_vz;
  replay_fusion_config.ekf_q_theta = config.stage_e_fusion.ekf_q_theta;
  replay_fusion_config.ekf_q_theta_dot = config.stage_e_fusion.ekf_q_theta_dot;
  replay_fusion_config.ekf_r_radar_z = config.stage_e_fusion.ekf_r_radar_z;
  replay_fusion_config.ekf_r_radar_vz = config.stage_e_fusion.ekf_r_radar_vz;
  replay_fusion_config.ekf_r_cam_theta = config.stage_e_fusion.ekf_r_cam_theta;
  replay_fusion_config.ekf_r_cam_z_weak =
      config.stage_e_fusion.ekf_r_cam_z_weak;
  g_active_radar_tx_offset_m.store(replay_fusion_config.radar_tx_m,
                                   std::memory_order_relaxed);

  adas::FCWMonitor::Config fcw_config;
  fcw_config.ttc_threshold_s = 3.0f;
  fcw_config.use_physics_fcw = true;
  fcw_config.friction_coefficient = 0.7f;
  fcw_config.reaction_time_s = 2.5f;
  fcw_config.min_fusion_quality = 0.2f;
  fcw_config.path_half_width_m = 0.9f;
  fcw_config.path_width_growth_per_m = 0.035f;
  fcw_config.warn_risk_threshold = 0.56f;
  fcw_config.critical_risk_threshold = 0.74f;
  fcw_config.ttc_last_ditch_s = 0.85f;
  fcw_config.ttc_immediate_warn_s = 2.8f;
  fcw_config.ttc_immediate_critical_s = 1.2f;
  fcw_config.camera_hold_ms = replay_fusion_config.camera_hold_ms;
  fcw_config.camera_drop_track_hold_ms = 1200;
  fcw_config.camera_drop_radar_recent_ms = 150;
  fcw_config.camera_drop_min_quality = 0.32f;
  fcw_config.invalid_demote_grace_ms = 350;
  fcw_config.invalid_state_hold_ms = 250;
  fcw_config.log_fcw_drop_reasons = true;
  fcw_config.min_trigger_object_speed_mps =
      std::max(0.0f, g_fcw_min_trigger_object_speed_gate_mps);
  applyStageESensitivity(replay_fusion_config, fcw_config,
                         g_stage_e_sensitivity_level);
  applyConfiguredFusionHoldTimings(replay_fusion_config, fcw_config,
                                   config.stage_e_fusion);
  g_sensor_fusion = std::make_unique<adas::SensorFusion>(replay_fusion_config);
  g_fcw_monitor = std::make_unique<adas::FCWMonitor>(fcw_config);

  g_ego_frame = std::make_unique<adas::EgoFrame>();
  g_ego_frame->init();

  std::ostringstream sens_replay_ss;
  sens_replay_ss << std::fixed << std::setprecision(2)
                 << sensitivityFactor(g_stage_e_sensitivity_level);
  std::cout << "[Main] Stage E fusion initialized (Replay Mode, Sensitivity: L"
            << g_stage_e_sensitivity_level << " ("
            << sensitivityLabel(g_stage_e_sensitivity_level) << ", x"
            << sens_replay_ss.str() << "), Min Trigger Speed Gate: "
            << fcw_config.min_trigger_object_speed_mps << " m/s)\n";

  // Initialize BLE Server (No real GPS connection needed for playback scaling,
  // but kept for UI output)
  g_ble_server = std::make_unique<adas::SimpleBleServer>();
  if (g_ble_server->initialize()) {
    g_ble_server->startAdvertising();
  }

  g_pipeline_running.store(true);
  g_pipeline_start_time = std::chrono::steady_clock::now();

  g_status_running.store(true);
  g_status_thread = std::thread(statusBarThread);

  g_visualizer_running.store(true);
  g_visualizer_thread = std::thread(visualizationThread);

  std::cout << "\n[Main] Replay Pipeline started successfully!\n";
  std::cout << "[Main] Replaying: " << replay_file << " at " << speed
            << "x speed\n";
  std::cout << "[Main] Press '3' to view status, '2' to stop\n\n";
}

void stopPipeline() {
  if (!g_pipeline_running.load()) {
    std::cout << "[Main] Pipeline is not running\n";
    return;
  }

  // Dump metrics if logging was enabled
  if (g_metrics_logger && g_metrics_logger->isEnabled()) {
    // Generate timestamp for filename
    auto now = std::chrono::system_clock::now();
    auto timestamp =
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch())
            .count();

    // Get Desktop path (works on Linux/Jetson)
    const char *home = std::getenv("HOME");
    if (!home) {
      home = "/home/ubuntu"; // Fallback
    }

    std::string desktop_path = std::string(home) + "/Desktop";
    std::string metrics_file =
        desktop_path + "/adas_metrics_" + std::to_string(timestamp) + ".csv";

    std::cout << "[Main] Saving metrics to: " << metrics_file << "\n";
    if (g_metrics_logger->dumpToCSV(metrics_file)) {
      std::cout << "[Main] Metrics saved successfully ("
                << g_metrics_logger->getEntryCount() << " entries)\n";
    } else {
      std::cerr << "[Main] Failed to save metrics\n";
    }
  }

  // Stop recording if active
  if (g_recorder && g_recorder->isRecording()) {
    g_recorder->stop();
    g_ingest_manager->setRecorder(nullptr);
  }

  // Stop BLE
  if (g_ble_server) {
    g_ble_server->shutdown();
    g_ble_server.reset();
  }

  std::cout << "\n[Main] Stopping pipeline...\n";
  stopDeepStreamApp();

  // Stop status bar thread first
  g_status_running.store(false);
  if (g_status_thread.joinable()) {
    g_status_thread.join();
  }

  // Stop visualization thread
  g_visualizer_running.store(false);
  if (g_visualizer_thread.joinable()) {
    g_visualizer_thread.join();
  }

  if (g_range_distance_capture_logger.isEnabled()) {
    const auto capture_path = g_range_distance_capture_logger.getFilePath();
    const auto sample_count = g_range_distance_capture_logger.getSampleCount();
    g_range_distance_capture_logger.stop();
    std::cout << "[Main] Camera/radar range capture stopped. File: "
              << capture_path << " | Samples: " << sample_count << "\n";
  }

  // Stop in reverse order
  if (g_bev_dashboard) {
    g_bev_dashboard->stop();
    g_bev_dashboard.reset();
  }

  if (g_bsd_receiver) {
    g_bsd_receiver->stop();
    g_bsd_receiver.reset();
  }

  if (g_ingest_manager) {
    g_ingest_manager->stop();
    g_ingest_manager.reset();
  }

  g_pipeline_running.store(false);
  std::cout << "[Main] Pipeline stopped\n";
}

void showStatus() {
  if (!g_pipeline_running.load()) {
    std::cout << "\n[Main] Pipeline is not running\n";
    return;
  }

  if (g_ingest_manager) {
    g_ingest_manager->printStatus();
  }
}

} // namespace

// Main entry point for the Jetson ADAS Core application
// Triggers CI rebuild
int main(int argc, char **argv) {
  printBanner();

  // Setup signal handlers
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // Parse command line args
  std::string config_path = "config/componentConfig.yaml";
  std::string hw_map_path = "config/hardware_map.json";
  std::string calib_dir = "config/calibration";
  bool auto_start = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--hardware-map" && i + 1 < argc) {
      hw_map_path = argv[++i];
    } else if (arg == "--calib-dir" && i + 1 < argc) {
      calib_dir = argv[++i];
    } else if (arg == "--auto-start") {
      auto_start = true;
    } else if (arg == "--help") {
      std::cout
          << "Usage: " << argv[0] << " [options]\n"
          << "Options:\n"
          << "  --config <path>        Path to componentConfig.yaml\n"
          << "  --hardware-map <path>  Path to hardware_map.json\n"
          << "  --calib-dir <path>     Path to calibration directory\n"
          << "  --auto-start           Start pipeline automatically\n"
          << "  --record <dir>         Record sensor data to directory\n"
          << "  --replay <file>        Replay from .adasrec file\n"
          << "  --replay-speed <float> Replay speed multiplier (default: 1.0)\n"
          << "  --replay-fast          Replay as fast as possible\n"
          << "  --help                 Show this help\n";
      return 0;
    } else if (arg == "--record" && i + 1 < argc) {
      g_record_dir = argv[++i];
    } else if (arg == "--replay" && i + 1 < argc) {
      g_replay_file = argv[++i];
    } else if (arg == "--replay-speed" && i + 1 < argc) {
      g_replay_speed = std::stof(argv[++i]);
    } else if (arg == "--replay-fast") {
      g_replay_fast = true;
    }
  }

  try {
    // Check if hardware map exists
    if (!adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
      std::cout << "[Main] Hardware map not found at " << hw_map_path << "\n";
      std::cout << "[Main] Please run Device Wizard first (option 4)\n\n";
    }

    // Load configuration
    std::cout << "[Main] Loading configuration from: " << config_path << "\n";
    adas::Config config = adas::ConfigLoader::loadConfig(config_path);
    config.front_radar.output_mode =
        normalizeRadarOutputMode(config.front_radar.output_mode);

    // Load hardware mapping (may be empty if file doesn't exist)
    adas::HardwareMap hw_map;
    if (adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
      std::cout << "[Main] Loading hardware map from: " << hw_map_path << "\n";
      hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);

      std::cout << "[Main] Mapped devices:\n";
      for (const auto &[mount, path] : hw_map.mappings) {
        std::cout << "  " << adas::mountToString(mount) << " -> " << path
                  << "\n";
      }
    }

    // Auto-start if requested
    if (auto_start && !hw_map.mappings.empty()) {
      startPipeline(config, hw_map);
    }

    // Interactive menu loop
    while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
      printMenu(config);

      std::string input;
      std::cin >> input;

      if (input == "h" || input == "H") {
        std::cout << "  Enter new camera height from road (meters): ";
        float new_height = 0.0f;
        std::cin >> new_height;
        if (!std::cin.fail() && new_height > 0.0f) {
          config.mounts[adas::Mount::FrontCam].xyz_m[2] = new_height;
          try {
            adas::ConfigLoader::saveConfig(config_path, config);
            if (g_sensor_fusion) {
              g_sensor_fusion->setCameraHeight(new_height);
              std::cout << "\n[Config] Dynamic camera height updated to "
                        << new_height << "m on-the-fly!\n\n";
            } else {
              std::cout << "\n[Config] Camera height saved to config (will "
                           "apply on next start).\n\n";
            }
          } catch (const std::exception &e) {
            std::cout << "\n[Error] Failed to save config: " << e.what()
                      << "\n\n";
          }
        } else {
          std::cin.clear();
          std::cout << "\n[Error] Invalid height value.\n\n";
        }
        std::cin.ignore(10000, '\n');
        continue;
      }

      int choice = -1;
      try {
        choice = std::stoi(input);
      } catch (...) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        continue;
      }

      switch (choice) {
      case 1: // Start Pipeline
        // Reload hw_map in case wizard was run
        if (adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
          hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);
        }
        if (hw_map.mappings.empty()) {
          std::cout << "[Main] No devices mapped. Run Device "
                       "Wizard first "
                       "(option 4)\n";
        } else {
          startPipeline(config, hw_map);
        }
        break;

      case 2: // Stop Pipeline
        stopPipeline();
        break;

      case 3: // Show Status
        showStatus();
        break;

      case 4: // Device Wizard
        if (g_pipeline_running.load()) {
          std::cout << "[Main] Please stop the pipeline first\n";
        } else {
          adas::DeviceWizard::runRegistration(hw_map_path);
        }
        break;

      case 5: // Camera Calibration
        if (g_pipeline_running.load()) {
          std::cout << "[Main] Please stop the pipeline first\n";
        } else {
          // Reload hw_map and run calibration
          if (adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
            hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);
          }
          adas::DeviceWizard::runCalibration(hw_map, calib_dir);
        }
        break;

      case 6: // Register Pi4 Network Devices
        if (g_pipeline_running.load()) {
          std::cout << "[Main] Please stop the pipeline first\n";
        } else {
          // Hardcoded Pi IP for static ethernet-to-ethernet
          // connection
          adas::DeviceWizard::registerNetworkDevices(hw_map_path,
                                                     "192.168.55.2");
        }
        break;

      case 7: // Test RTT to Pi4
      {
        std::cout << "  Enter Pi4 IP address: ";
        std::string pi_ip;
        std::cin >> pi_ip;
        std::cin.ignore(10000, '\n');
        double rtt = adas::DeviceWizard::measureRTT(pi_ip);
        if (rtt < 0) {
          std::cout << "[RTT] Failed to reach " << pi_ip << "\n";
        } else {
          std::cout << "[RTT] Round-trip time to " << pi_ip << ": " << rtt
                    << " ms\n";
          if (rtt < 10) {
            std::cout << "[RTT] Status: EXCELLENT (< 10ms)\n";
          } else if (rtt < 25) {
            std::cout << "[RTT] Status: GOOD (< 25ms)\n";
          } else if (rtt < 50) {
            std::cout << "[RTT] Status: ACCEPTABLE (< 50ms)\n";
          } else {
            std::cout << "[RTT] Status: WARNING - High latency "
                         "may affect sync\n";
          }
        }
      } break;

      case 8: // Toggle Verbose Mode
      {
        bool new_state = !adas::g_verbose_mode.load();
        adas::g_verbose_mode.store(new_state);
        std::cout << "[Main] Verbose mode "
                  << (new_state ? "ENABLED" : "DISABLED") << "\n";
      } break;

      case 9: // View Undistortion Demo
        if (g_pipeline_running.load()) {
          std::cout << "[Main] Please stop the pipeline first\n";
        } else {
          std::cout << "[Main] Launching undistortion demo...\n";
          std::cout << "[Main] Press 'q' in the OpenCV window to "
                       "exit.\n";
          int ret = std::system("python3 scripts/view_undistortion.py 0");
          if (ret != 0) {
            std::cout << "[Main] Demo script exited with code " << ret << "\n";
          }
        }
        break;

      case 10: // Toggle Metrics Logging
      {
        if (!g_metrics_logger) {
          g_metrics_logger = std::make_unique<adas::MetricsLogger>();
        }

        if (g_metrics_logger->isEnabled()) {
          g_metrics_logger->disable();
        } else {
          g_metrics_logger->enable();
          g_metrics_logger->clear(); // Clear old data when enabling
          std::cout << "[Main] Metrics logging started. Data "
                       "will be saved "
                       "to Desktop on "
                       "pipeline stop.\n";
        }
      } break;

      case 11: // Toggle Recording
      {
        if (!g_pipeline_running.load()) {
          std::cout << "[Main] Start the pipeline first\n";
        } else if (g_recorder && g_recorder->isRecording()) {
          // Stop recording
          g_recorder->stop();
          g_ingest_manager->setRecorder(nullptr);
          std::cout << "\n";
          std::cout << "======================================================="
                       "=======\n";
          std::cout << "  ⏹  RECORDING STOPPED                                 "
                       "       \n";
          std::cout << "======================================================="
                       "=======\n";
          std::cout << "  File: " << g_recorder->getFilePath() << "\n";
          std::cout << "  Events: " << g_recorder->getEventCount() << "\n";
          std::cout << "======================================================="
                       "=======\n\n";
        } else {
          // Start recording
          if (!g_recorder) {
            g_recorder = std::make_unique<adas::Recorder>();
          }
          if (g_recorder->start(g_record_dir)) {
            g_ingest_manager->setRecorder(g_recorder.get());
            std::cout << "\n";
            std::cout << "====================================================="
                         "=========\n";
            std::cout << "  🔴 RECORDING STARTED — ALL SENSORS ACTIVE          "
                         "         \n";
            std::cout << "====================================================="
                         "=========\n";
            std::cout << "  File: " << g_recorder->getFilePath() << "\n";
            std::cout << "  Dir:  " << g_record_dir << "\n";
            std::cout << "  Press 11 again to STOP recording.\n";
            std::cout << "====================================================="
                         "=========\n\n";
          } else {
            std::cerr << "[Main] Failed to start recording\n";
          }
        }
      } break;

      case 13: // Edit Camera Config
      {
        std::cout << "\n[Config] Opening componentConfig.yaml in editor...\n";
        std::cout << "[Config] Save and exit the editor to apply changes.\n\n";

        // Use EDITOR env var, fall back to nano then vi
        const char *editor_env = std::getenv("EDITOR");
        std::string editor = editor_env ? editor_env : "nano";
        std::string cmd = editor + " " + config_path;
        int ret = std::system(cmd.c_str());
        if (ret != 0) {
          // Try vi as last resort
          cmd = "vi " + config_path;
          ret = std::system(cmd.c_str());
        }

        // Reload config from disk
        try {
          config = adas::ConfigLoader::loadConfig(config_path);
          config.front_radar.output_mode =
              normalizeRadarOutputMode(config.front_radar.output_mode);
          std::cout << "\n[Config] Reloaded from: " << config_path << "\n";
          std::cout << "[Config] Camera: front=" << config.cameras.width << "x"
                    << config.cameras.height
                    << " | fps=" << config.cameras.target_fps
                    << " | mjpeg=" << (config.cameras.use_mjpeg ? "yes" : "NO")
                    << "\n";

          // Hot-reload: if pipeline is running, restart ingest to apply new
          // camera settings. Fusion and alerting keep running; only ingest
          // restarts.
          if (g_pipeline_running.load() && g_ingest_manager) {
            std::cout << "[Config] Pipeline is running. Restarting camera "
                         "ingest to apply new settings...\n";
            g_ingest_manager->stop();
            g_ingest_manager =
                std::make_unique<adas::IngestManager>(config, hw_map);
            g_ingest_manager->start();
            if (g_recorder && g_recorder->isRecording()) {
              // Re-wire recorder after restart once receivers exist again.
              g_ingest_manager->setRecorder(g_recorder.get());
            }
            std::cout << "[Config] Camera ingest restarted with new config.\n";
          } else {
            std::cout << "[Config] Changes will take effect on next pipeline "
                         "start.\n";
          }
        } catch (const std::exception &e) {
          std::cerr << "[Config] Failed to reload config: " << e.what() << "\n";
          std::cerr << "[Config] Previous config is still active.\n";
        }
      } break;

      case 12: // Start Replay Pipeline
      {
        if (g_pipeline_running.load()) {
          std::cout << "[Main] Please stop the pipeline first\n";
        } else {
          std::string file_path;
          std::string speed_str;
          float speed = 1.0f;

          std::cout << "  Enter path to .adasrec file: ";
          std::cin >> file_path;
          std::cin.ignore(10000, '\n'); // Clear newline

          std::cout << "  Enter playback speed (e.g., 1.0, 0.5, 2.0) [1.0]: ";
          std::getline(std::cin, speed_str);
          if (!speed_str.empty()) {
            try {
              speed = std::stof(speed_str);
              if (speed <= 0.0f) {
                std::cout << "[Main] Invalid speed. Defaulting to "
                             "fast-as-possible.\n";
                // `speed <= 0.0f` is fast mode per ReplayEngine
              }
            } catch (...) {
              std::cout << "[Main] Invalid number, defaulting to 1.0x\n";
              speed = 1.0f;
            }
          }

          if (adas::ConfigLoader::hardwareMapExists(hw_map_path)) {
            hw_map = adas::ConfigLoader::loadHardwareMap(hw_map_path);
          }
          startReplayPipeline(file_path, speed, config, hw_map);
        }
      } break;

      case 15: // Set Stage E Sensitivity
      {
        std::cout << "  Enter Stage E sensitivity [1-5] (1=least, 5=most): ";
        int level = g_stage_e_sensitivity_level;
        std::cin >> level;
        if (std::cin.fail() || level < 1 || level > 5) {
          std::cin.clear();
          std::cin.ignore(10000, '\n');
          std::cout << "[Main] Invalid sensitivity level. Use 1..5.\n";
          break;
        }
        g_stage_e_sensitivity_level = clampSensitivityLevel(level);
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << sensitivityFactor(g_stage_e_sensitivity_level);
        std::cout << "[Main] Stage E sensitivity set to L"
                  << g_stage_e_sensitivity_level << " ("
                  << sensitivityLabel(g_stage_e_sensitivity_level) << ", x"
                  << ss.str() << ")";
        if (g_pipeline_running.load()) {
          std::cout << " - will apply on next pipeline start (stop/start).";
        }
        std::cout << "\n";
      } break;

      case 16: // Set Front Radar Output Mode
      {
        std::cout << "  Select front radar output mode:\n";
        std::cout << "    1) split_range (legacy split speed/range stream)\n";
        std::cout
            << "    2) combined_native (single packet speed+range via OY)\n";
        std::cout << "  Enter choice [1-2]: ";

        int mode_choice = 0;
        std::cin >> mode_choice;
        if (std::cin.fail() || (mode_choice != 1 && mode_choice != 2)) {
          std::cin.clear();
          std::cin.ignore(10000, '\n');
          std::cout << "[Main] Invalid mode choice. Use 1 or 2.\n";
          break;
        }

        config.front_radar.output_mode =
            (mode_choice == 2) ? "combined_native" : "split_range";
        try {
          adas::ConfigLoader::saveConfig(config_path, config);
          std::cout << "[Main] Front radar output mode set to "
                    << radarOutputModeLabel(config.front_radar.output_mode)
                    << ".";
          if (g_pipeline_running.load()) {
            std::cout << " Applies on next pipeline start (stop/start).";
          }
          std::cout << "\n";
        } catch (const std::exception &e) {
          std::cerr << "[Main] Failed to save front radar output mode: "
                    << e.what() << "\n";
        }
      } break;

      case 17: // Set FCW Min Trigger Object Speed Gate
      {
        std::cout << "  Enter FCW min trigger object speed gate (m/s, >= 0): ";
        float speed_gate_mps = g_fcw_min_trigger_object_speed_gate_mps;
        std::cin >> speed_gate_mps;
        if (std::cin.fail() || !std::isfinite(speed_gate_mps) ||
            speed_gate_mps < 0.0f) {
          std::cin.clear();
          std::cin.ignore(10000, '\n');
          std::cout << "[Main] Invalid speed gate value. Use a number >= 0.\n";
          break;
        }

        g_fcw_min_trigger_object_speed_gate_mps = speed_gate_mps;
        if (g_fcw_monitor) {
          g_fcw_monitor->setMinTriggerObjectSpeedGateMps(
              g_fcw_min_trigger_object_speed_gate_mps);
        }

        std::cout << "[Main] FCW min trigger object speed gate set to "
                  << g_fcw_min_trigger_object_speed_gate_mps << " m/s";
        if (g_pipeline_running.load() && g_fcw_monitor) {
          std::cout << " (applied on-the-fly).";
        } else {
          std::cout << " (will apply on next pipeline start).";
        }
        std::cout << "\n";
      } break;

      case 18: // Ubuntu Camera Node Swap Hotfix
        runUbuntuCameraNodeSwapHotfix();
        break;

      case 19: // Toggle Cam/Radar Range Capture
      {
        if (!g_pipeline_running.load()) {
          std::cout << "[Main] Start the pipeline first\n";
          break;
        }

        if (g_range_distance_capture_logger.isEnabled()) {
          const auto capture_path =
              g_range_distance_capture_logger.getFilePath();
          const auto sample_count =
              g_range_distance_capture_logger.getSampleCount();
          g_range_distance_capture_logger.stop();
          std::cout << "[Main] Camera/radar range capture stopped.\n";
          std::cout << "[Main] File: " << capture_path << "\n";
          std::cout << "[Main] Samples: " << sample_count << "\n";
        } else {
          if (g_range_distance_capture_logger.start()) {
            std::cout << "[Main] Camera/radar range capture started.\n";
            std::cout << "[Main] File: "
                      << g_range_distance_capture_logger.getFilePath() << "\n";
            std::cout << "[Main] Logging matched fused objects with:\n";
            std::cout << "        fusion_quality >= 0.60, |theta| <= 10 deg,\n";
            std::cout << "        camera_age <= 150 ms, radar_age <= 150 ms,\n";
            std::cout << "        classes in {Car, Bicycle, Person},\n";
            std::cout << "        has radar and non-predicted camera range.\n";
          } else {
            std::cerr << "[Main] Failed to start camera/radar range capture\n";
          }
        }
      } break;

      case 20: // Set Fusion Hold Timings
      {
        int camera_hold_ms = config.stage_e_fusion.camera_hold_ms;
        int radar_hold_ms = config.stage_e_fusion.radar_hold_ms;
        int track_cleanup_ms = config.stage_e_fusion.track_cleanup_ms;
        int predicted_camera_threshold_ms =
            config.stage_e_fusion.predicted_camera_threshold_ms;

        std::cout << "  Enter camera consistency hold (ms) [current "
                  << camera_hold_ms << "]: ";
        std::cin >> camera_hold_ms;
        std::cout << "  Enter published radar-range hold (ms) [current "
                  << radar_hold_ms << "]: ";
        std::cin >> radar_hold_ms;
        std::cout << "  Enter track cleanup hold (ms) [current "
                  << track_cleanup_ms << "]: ";
        std::cin >> track_cleanup_ms;
        std::cout << "  Enter predicted-camera threshold (ms) [current "
                  << predicted_camera_threshold_ms << "]: ";
        std::cin >> predicted_camera_threshold_ms;

        if (std::cin.fail()) {
          std::cin.clear();
          std::cin.ignore(10000, '\n');
          std::cout << "[Main] Invalid timing input.\n";
          break;
        }

        std::string validation_error;
        if (!validateFusionHoldTimings(
                camera_hold_ms, radar_hold_ms, track_cleanup_ms,
                predicted_camera_threshold_ms, validation_error)) {
          std::cout << "[Main] Invalid fusion hold timings: "
                    << validation_error << "\n";
          break;
        }

        config.stage_e_fusion.camera_hold_ms = camera_hold_ms;
        config.stage_e_fusion.radar_hold_ms = radar_hold_ms;
        config.stage_e_fusion.track_cleanup_ms = track_cleanup_ms;
        config.stage_e_fusion.predicted_camera_threshold_ms =
            predicted_camera_threshold_ms;

        try {
          adas::ConfigLoader::saveConfig(config_path, config);
        } catch (const std::exception &e) {
          std::cerr << "[Main] Failed to save fusion hold timings: " << e.what()
                    << "\n";
          break;
        }

        if (g_pipeline_running.load()) {
          if (g_sensor_fusion) {
            g_sensor_fusion->setCameraHoldMs(
                static_cast<uint32_t>(camera_hold_ms));
            g_sensor_fusion->setRadarHoldMs(
                static_cast<uint32_t>(radar_hold_ms));
            g_sensor_fusion->setTrackCleanupMs(
                static_cast<uint32_t>(track_cleanup_ms));
            g_sensor_fusion->setPredictedCameraThresholdMs(
                static_cast<uint32_t>(predicted_camera_threshold_ms));
          }
          if (g_fcw_monitor) {
            g_fcw_monitor->setCameraHoldMs(
                static_cast<uint32_t>(camera_hold_ms));
          }
          if (g_bev_dashboard) {
            g_bev_dashboard->setDeadTrackCleanupMs(
                static_cast<uint32_t>(track_cleanup_ms));
          }
          std::cout << "[Main] Fusion hold timings updated on-the-fly and "
                       "saved.\n";
        } else {
          std::cout << "[Main] Fusion hold timings saved. They will apply on "
                       "next pipeline start.\n";
        }

        std::cout << "        camera_hold_ms=" << camera_hold_ms
                  << ", radar_hold_ms=" << radar_hold_ms
                  << ", track_cleanup_ms=" << track_cleanup_ms
                  << ", predicted_camera_threshold_ms="
                  << predicted_camera_threshold_ms << "\n";
      } break;

      case 14: // Toggle RTSP Server
      {
        bool new_val = !g_rtsp_streaming.load();
        g_rtsp_streaming.store(new_val);
        std::cout << "[Main] RTSP Streaming "
                  << (new_val ? "ENABLED" : "DISABLED")
                  << " (takes effect on next pipeline start)\n";
      } break;

      case 0: // Exit
        stopPipeline();
        std::cout << "\n[Main] Goodbye!\n";
        return 0;

      default:
        std::cout << "[Main] Invalid choice\n";
        break;
      }
    }

    // Graceful shutdown on signal
    stopPipeline();
    std::cout << "[Main] Shutdown complete.\n";
    return 0;

  } catch (const std::exception &e) {
    std::cerr << "[Main] Fatal error: " << e.what() << "\n";
    return 1;
  }
}
