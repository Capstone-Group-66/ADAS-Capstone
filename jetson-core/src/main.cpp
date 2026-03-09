// File: src/main.cpp
// ADAS Pipeline Entry Point - Interactive CLI with Stages A, B, E
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
// POSIX process management (fork/exec/kill) for deepstream_fusion.py
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
#include "adas/recording/ReplayEngine.hpp"
#include "adas/stage_a/DeviceWizard.hpp"
#include "adas/stage_a/IngestManager.hpp"
#include "adas/stage_b/CameraPipeline.hpp"
// FrontCamDeepStream.hpp removed — pipeline runs as deepstream_fusion.py
// (Python pyds)

#include "adas/stage_b/ObjectDetector.hpp" // For class name lookup
#include "adas/stage_e/EgoFrame.hpp"
#include "adas/stage_e/FCWMonitor.hpp"
#include "adas/stage_e/SensorFusion.hpp"

namespace {

// Global managers
std::unique_ptr<adas::IngestManager> g_ingest_manager;
std::unique_ptr<adas::StageBManager> g_stage_b_manager;
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

// Detection output queues (Stage B -> Stage E)
// NOTE: g_det_front_queue is now fed directly by the FrontCamDeepStream pad
// probe (via IngestManager::getFrontCamDetQueue() backed by the same object).
// The visualisation thread reads from this queue exactly as before — no
// changes needed in Stage E. Side queues remain available for future BSD/LCW.
adas::SPSCQueue<adas::DetBatch, 8> g_det_front_queue; // Fed by DeepStream probe
adas::SPSCQueue<adas::DetBatch, 8> g_det_side_l_queue;
adas::SPSCQueue<adas::DetBatch, 8> g_det_side_r_queue;
adas::SPSCQueue<adas::DetBatch, 8> g_det_rear_queue;

// Radar data queue (Stage A -> Stage E)
adas::SPSCQueue<adas::RadarTargets, 4> g_radar_front_queue;

// Stage E: Fusion + FCW + EgoFrame
std::unique_ptr<adas::SensorFusion> g_sensor_fusion;
std::unique_ptr<adas::FCWMonitor> g_fcw_monitor;
std::unique_ptr<adas::EgoFrame> g_ego_frame;
std::atomic<bool> g_fcw_alert_active{false};
std::atomic<int> g_fcw_ttc_ms{
    0}; // TTC in milliseconds (avoids float atomic availability issues)

// BLE Server for mobile app communication
std::unique_ptr<adas::SimpleBleServer> g_ble_server;

// Metrics logger for validation
std::unique_ptr<adas::MetricsLogger> g_metrics_logger;

// Recording and replay
std::unique_ptr<adas::Recorder> g_recorder;
std::unique_ptr<adas::ReplayEngine> g_replay_engine;
std::string g_record_dir = "./recordings";
std::string g_replay_file;
float g_replay_speed = 1.0f;
bool g_replay_fast = false;

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

// Visualization control flag (set to false for production)
std::atomic<bool> g_visualizer_enabled{true};

// Stage E thread: fusion, alerting, BLE, metrics, and (optionally)
// visualization
void visualizationThread() {
  const bool display_enabled = g_visualizer_enabled.load();
  std::cout << "[StageE] Thread started (display "
            << (display_enabled ? "enabled" : "disabled") << ")\n";

  // Only create the Stage B window if there's a camera pipeline feeding it.
  // (FrontCam is handled by deepstream_fusion.py; this window is for future
  //  side-camera inference.)
  const bool window_wanted = display_enabled;
  bool window_created = false;

  if (window_wanted) {
    // Window is created lazily on first frame so we don't get a blank black box
  }

  auto last_fps_time = std::chrono::steady_clock::now();
  auto last_display_time = std::chrono::steady_clock::now();
  auto fcw_alert_until = std::chrono::steady_clock::now(); // FCW hold timer
  const auto display_interval =
      std::chrono::milliseconds(50); // 20 FPS cap for display
  const auto fcw_hold_duration =
      std::chrono::seconds(2); // Hold FCW alert for 2 seconds
  const float fcw_proximity_threshold_m =
      1.5f; // Trigger FCW if object within this range
  int frame_count = 0;
  double fps = 0.0;
  float last_fcw_ttc = 0.0f;
  float last_fcw_range = 0.0f;

  while (g_visualizer_running.load() && !g_shutdown_requested.load()) {
    bool got_frame = false;
    adas::DetBatch batch;
    // Drain to latest
    if (g_ingest_manager) {
      while (g_ingest_manager->getFrontCamDetQueue().try_pop(batch)) {
        got_frame = true;
      }
    }

    if (display_enabled && got_frame && !batch.frame.empty() &&
        !window_created) {
      cv::namedWindow("Stage B: FrontCam", cv::WINDOW_AUTOSIZE);
      window_created = true;
    }

    // Get latest radar data from IngestManager
    adas::RadarTargets radar;
    if (g_ingest_manager) {
      try {
        auto &radar_queue =
            g_ingest_manager->getRadarQueue(adas::Mount::FrontRadar);
        while (radar_queue.try_pop(radar)) {
          // Keep draining to get latest
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
        // Calculate dt from timestamp
        float dt = 0.01f; // Default 100Hz
        if (g_ego_frame->previous_time_ns != 0 &&
            imu_sample.t_capture > g_ego_frame->previous_time_ns) {
          dt = adas::Clock::ns_to_sec(imu_sample.t_capture -
                                      g_ego_frame->previous_time_ns);
        }
        g_ego_frame->previous_time_ns = imu_sample.t_capture;
        g_ego_frame->update(imu_sample, dt);
      }

      // Pass ego velocity to FCW monitor
      if (g_fcw_monitor) {
        g_fcw_monitor->setEgoVelocity(g_ego_frame->getForwardVelocity_mps());
      }
    }

    if (got_frame) {
      // Run Stage E fusion
      std::vector<adas::FusedObject> fused;
      if (g_sensor_fusion) {
        // Update fusion with the latest IMU pitch angle from the Pi.
        // The ZMQ imuThread writes this atomically; we read it here once per
        // frame so all distance estimates in this batch share the same θ.
        if (g_ingest_manager) {
          g_sensor_fusion->setPitch(g_ingest_manager->getLatestPitch());
        }
        fused = g_sensor_fusion->fuse(batch, radar);
      }

      // Velocity Deadband: Clamp speeds < 1 m/s to 0 to prevent
      // drift/noise Logic Assumption: Positive = Moving Away, Negative =
      // Moving Towards
      for (auto &obj : fused) {
        if (std::abs(obj.radial_vel_mps) < 1.0f) {
          obj.radial_vel_mps = 0.0f;
        }
      }

      // Check for FCW alerts (TTC-based)
      std::optional<adas::FCWAlert> fcw_alert;
      if (g_fcw_monitor && !fused.empty()) {
        fcw_alert = g_fcw_monitor->check(fused, adas::Clock::now_ns());
      }

      // Also check for proximity-based FCW (any object within 1.5m)
      bool proximity_alert = false;
      float closest_range = 999.0f;
      const adas::FusedObject *prox_obj =
          nullptr; // Capture the object causing the alert

      for (const auto &obj : fused) {
        if (obj.has_radar && obj.range_m < fcw_proximity_threshold_m &&
            obj.range_m > 0.1f) {
          proximity_alert = true;
          if (obj.range_m < closest_range) {
            closest_range = obj.range_m;
            prox_obj = &obj;
          }
        }
      }

      // Update FCW hold timer if we have an alert
      auto now_time = std::chrono::steady_clock::now();
      if (fcw_alert.has_value() || proximity_alert) {
        fcw_alert_until = now_time + fcw_hold_duration;
        if (fcw_alert.has_value()) {
          last_fcw_ttc = fcw_alert->ttc_s;
          last_fcw_range = fcw_alert->range_m;
        } else {
          last_fcw_ttc = closest_range / 1.0f;
          last_fcw_range = closest_range;
        }
      }

      // BLE Transmission: Heartbeat (1Hz) + Alerts (Immediate)
      if (g_ble_server && g_ble_server->isConnected()) {
        static auto last_ble_send = std::chrono::steady_clock::time_point();

        // Fix: Include proximity_alert in alerting condition so
        // Radar-only alerts are sent
        bool is_alerting = fcw_alert.has_value() || proximity_alert;

        auto time_since = std::chrono::duration_cast<std::chrono::milliseconds>(
            now_time - last_ble_send);

        if (is_alerting || time_since.count() > 1000) {
          std::vector<adas::Alert> alerts_to_send;

          // Use GPS-corrected ego speed for the phone dashboard
          int speed_kmh = 0;
          if (g_ego_frame) {
            speed_kmh = static_cast<int>(g_ego_frame->getSpeed_mps() * 3.6f);
          }

          if (fcw_alert.has_value()) {
            auto alert = adas::FCWAlertAdapter::convert(*fcw_alert,
                                                        adas::Clock::now_ns());
            alerts_to_send.push_back(alert);
          } else if (proximity_alert) {
            // Synthetic Alert from Proximity Logic
            adas::Alert alert;
            alert.id = "prox_" + std::to_string(adas::Clock::now_ns());
            alert.type = adas::AlertType::FCW; // Map to FCW for Mobile App
                                               // (turns red)
            alert.severity = adas::Severity::Critical;
            alert.rationale = "Proximity Warning (< 1.5m)";
            alerts_to_send.push_back(alert);
          } else {
            // Heartbeat: No alerts
          }

          uint16_t tickId =
              static_cast<uint16_t>(adas::Clock::now_ns() / 50'000'000);

          // Encode Payload
          auto payload = adas::encodeTickPayloadToCbor(tickId, speed_kmh, 0, 0,
                                                       alerts_to_send);

          // Fragment and Send
          auto frames = adas::fragmentPayload(tickId, payload, 185);
          for (const auto &frame : frames) {
            g_ble_server->notifyAlertStream(frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }

          last_ble_send = now_time;
        }
      }

      // Log metrics if enabled
      if (g_metrics_logger && g_metrics_logger->isEnabled()) {
        uint64_t now_ns = adas::Clock::now_ns();
        double e2e_latency_ms = (now_ns - batch.h.t_ingest_ns) / 1e6;

        // Capture TTC and range from FCW alert if present
        float ttc = fcw_alert.has_value() ? fcw_alert->ttc_s : -1.0f;
        float range = fcw_alert.has_value() ? fcw_alert->range_m : -1.0f;
        bool triggered = fcw_alert.has_value() || proximity_alert;

        g_metrics_logger->logFrame(now_ns / 1e6, // timestamp_ms
                                   batch.h.seq,
                                   batch.inference_time_us /
                                       1000.0, // convert to ms
                                   ttc, range, triggered, e2e_latency_ms);
      }

      // ── Step 10: OpenCV Visualization (only when display is enabled and
      // frame exists) ──
      if (display_enabled && !batch.frame.empty()) {
        // CRITICAL: Clone the frame to get our own memory buffer
        // The original batch.frame may be reused by ingest thread
        cv::Mat vis = batch.frame.clone();
        int vis_width = vis.cols;
        int vis_height = vis.rows;

        // Detection persistence: Hold detections for 500ms to reduce jitter
        // Uses a static buffer that persists across frames
        static std::vector<
            std::pair<adas::FusedObject, std::chrono::steady_clock::time_point>>
            persistent_dets;
        const auto det_hold_duration = std::chrono::milliseconds(500);

        // Add/update current detections in buffer
        for (const auto &obj : fused) {
          // Check if similar detection exists (overlap > 50%)
          bool found = false;
          for (auto &[stored_obj, timestamp] : persistent_dets) {
            cv::Rect2f intersection = stored_obj.box_px & obj.box_px;
            float overlap =
                intersection.area() /
                std::max(stored_obj.box_px.area(), obj.box_px.area());
            if (overlap > 0.3f && stored_obj.object_class == obj.object_class) {
              // Update existing detection
              stored_obj = obj;
              timestamp = now_time;
              found = true;
              break;
            }
          }
          if (!found) {
            persistent_dets.push_back({obj, now_time});
          }
        }

        // Remove stale detections
        persistent_dets.erase(
            std::remove_if(persistent_dets.begin(), persistent_dets.end(),
                           [&](const auto &item) {
                             return now_time - item.second > det_hold_duration;
                           }),
            persistent_dets.end());

        // Draw persistent detections (instead of just current frame)
        for (const auto &[obj, timestamp] : persistent_dets) {
          // Bounds check - skip invalid detections
          int x = static_cast<int>(obj.box_px.x);
          int y = static_cast<int>(obj.box_px.y);
          int w = static_cast<int>(obj.box_px.width);
          int h = static_cast<int>(obj.box_px.height);

          // Clamp to frame bounds
          x = std::max(0, std::min(x, vis_width - 1));
          y = std::max(0, std::min(y, vis_height - 1));
          w = std::max(1, std::min(w, vis_width - x));
          h = std::max(1, std::min(h, vis_height - y));

          cv::Rect safe_box(x, y, w, h);
          cv::Point safe_centroid(
              std::max(0, std::min(static_cast<int>(obj.centroid_px.x),
                                   vis_width - 1)),
              std::max(0, std::min(static_cast<int>(obj.centroid_px.y),
                                   vis_height - 1)));

          cv::Scalar color((obj.object_class * 50) % 255,
                           (obj.object_class * 80 + 100) % 255,
                           (obj.object_class * 120 + 200) % 255);

          cv::rectangle(vis, safe_box, color, 2);

          std::string class_name =
              adas::ObjectDetector::getClassName(obj.object_class);
          std::string label =
              class_name + " " +
              std::to_string(static_cast<int>(obj.score * 100)) + "%";

          // Add TTC if radar matched
          if (obj.has_radar) {
            if (obj.ttc_s < 100.0f) {
              label += " R:" + std::to_string(static_cast<int>(obj.range_m)) +
                       "m TTC:" + std::to_string(static_cast<int>(obj.ttc_s)) +
                       "s";
            } else {
              label +=
                  " R:" + std::to_string(static_cast<int>(obj.range_m)) + "m";
            }
          }

          int baseLine;
          cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                               0.5, 1, &baseLine);

          int label_y = std::max(labelSize.height + 2, y);

          cv::rectangle(
              vis, cv::Point(x, label_y - labelSize.height - 2),
              cv::Point(std::min(x + labelSize.width, vis_width), label_y),
              color, cv::FILLED);

          cv::putText(vis, label, cv::Point(x, label_y - 2),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

          cv::circle(vis, safe_centroid, 3, cv::Scalar(0, 255, 0), -1);
        }

        // Draw FCW alert overlay if active (with 2-second hold)
        if (now_time < fcw_alert_until) {
          g_fcw_alert_active.store(true);
          g_fcw_ttc_ms.store(static_cast<int>(last_fcw_ttc * 1000.0f));

          // Red border
          if (vis.cols > 10 && vis.rows > 10) {
            cv::rectangle(vis, cv::Point(4, 4),
                          cv::Point(vis.cols - 5, vis.rows - 5),
                          cv::Scalar(0, 0, 255), 8);
          }

          // FCW warning text with range
          std::string fcw_text =
              "FCW ALERT! Range:" +
              std::to_string(static_cast<int>(last_fcw_range * 10) / 10) + "." +
              std::to_string(static_cast<int>(last_fcw_range * 10) % 10) + "m";
          int text_x = std::max(10, vis.cols / 2 - 150);
          cv::putText(vis, fcw_text, cv::Point(text_x, 60),
                      cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);
        } else {
          g_fcw_alert_active.store(false);
        }

        // Calculate FPS
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_fps_time)
                           .count();
        if (elapsed >= 1000) {
          fps = frame_count * 1000.0 / elapsed;
          frame_count = 0;
          last_fps_time = now;
        }

        // Draw info overlay
        std::string info =
            "Inf: " +
            std::to_string(static_cast<int>(batch.inference_time_us / 1000)) +
            "ms | FPS: " + std::to_string(static_cast<int>(fps));
        cv::putText(vis, info, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 0), 2);

        // Rate-limit display to 20 FPS to reduce stuttering
        auto now_display = std::chrono::steady_clock::now();
        if (now_display - last_display_time >= display_interval) {
          cv::imshow("Stage B: FrontCam", vis);
          last_display_time = now_display;
        }
      } // end if (display_enabled) — Step 10
    }

    if (display_enabled) {
      // Non-blocking waitKey with minimal delay
      if (cv::waitKey(1) == 'q') {
        g_shutdown_requested.store(true);
      }
    }

    if (!got_frame) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  if (display_enabled) {
    try {
      cv::destroyAllWindows();
    } catch (...) {
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

void printMenu() {
  std::cout << "\n";
  std::cout
      << "==============================================================\n";
  std::cout
      << "                    MAIN MENU                                 \n";
  std::cout
      << "==============================================================\n";
  std::cout << "  1) Start Pipeline (Stages A + B)\n";
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
  std::cout << "  h) Set Camera Height On-The-Fly\n";
  std::cout << "  0) Exit\n";
  std::cout
      << "==============================================================\n";
  std::cout << "  Enter choice: ";
}

void startPipeline(const adas::Config &config, const adas::HardwareMap &hw_map,
                   const std::string &calib_dir,
                   const std::string &model_path) {
  if (g_pipeline_running.load()) {
    std::cout << "[Main] Pipeline is already running\n";
    return;
  }

  std::cout << "\n[Main] Starting pipeline...\n";

  // Stage A: Ingest
  g_ingest_manager = std::make_unique<adas::IngestManager>(config, hw_map);
  g_ingest_manager->start();

  // Stage B: Camera Preprocessing + Inference
  g_stage_b_manager =
      std::make_unique<adas::StageBManager>(calib_dir, model_path);

  // ── DeepStream: launch deepstream-app for FrontCam ─────────────────
  // The C application runs nvinfer (DashCamNet) inside GStreamer and shows an
  // nveglglessink window with bounding boxes. It connects to our ZMQ IPC socket
  // to feed detections.
  launchDeepStreamApp();

  // Side cameras can be added to Stage B here in future (BSD/LCW):
  g_stage_b_manager->start();

  // Stage E: Fusion + FCW + EgoFrame
  adas::FusionConfig fusion_config;
  auto it = config.mounts.find(adas::Mount::FrontCam);
  if (it != config.mounts.end()) {
    fusion_config.cam_height_m = it->second.xyz_m[2];
  }
  g_sensor_fusion = std::make_unique<adas::SensorFusion>(fusion_config);

  // Configure FCW with physics-based parameters
  adas::FCWMonitor::Config fcw_config;
  fcw_config.ttc_threshold_s = 3.0f;
  fcw_config.use_physics_fcw = true;      // Enable physics-based FCW
  fcw_config.friction_coefficient = 0.7f; // Dry asphalt
  fcw_config.reaction_time_s = 2.5f;      // Driver reaction time
  g_fcw_monitor = std::make_unique<adas::FCWMonitor>(fcw_config);

  // Initialize EgoFrame for ego vehicle state from IMU
  g_ego_frame = std::make_unique<adas::EgoFrame>();
  g_ego_frame->init();

  std::cout << "[Main] Stage E fusion initialized (TTC threshold: "
            << g_fcw_monitor->getThreshold() << "s, Physics FCW: "
            << (fcw_config.use_physics_fcw ? "ENABLED" : "disabled") << ")\n";

  // Initialize BLE Server
  g_ble_server = std::make_unique<adas::SimpleBleServer>();
  if (g_ble_server->initialize()) {
    // Wire GPS callback: phone GPS -> EgoFrame drift correction
    g_ble_server->setOnGpsData([](float speed_mps, uint64_t ts_ms) {
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
                         const adas::HardwareMap &hw_map,
                         const std::string &calib_dir,
                         const std::string &model_path) {
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

  // Stage B: Camera Preprocessing + Inference
  g_stage_b_manager =
      std::make_unique<adas::StageBManager>(calib_dir, model_path);

  // Wire FrontCam queue for inference visualizer
  if (hw_map.mappings.find(adas::Mount::FrontCam) != hw_map.mappings.end()) {
    g_stage_b_manager->addCamera(
        adas::Mount::FrontCam,
        g_ingest_manager->getCameraQueue(adas::Mount::FrontCam),
        g_det_front_queue);
  }
  g_stage_b_manager->start();

  // Stage E: Fusion + FCW + EgoFrame
  g_sensor_fusion = std::make_unique<adas::SensorFusion>();

  adas::FCWMonitor::Config fcw_config;
  fcw_config.ttc_threshold_s = 3.0f;
  fcw_config.use_physics_fcw = true;
  fcw_config.friction_coefficient = 0.7f;
  fcw_config.reaction_time_s = 2.5f;
  g_fcw_monitor = std::make_unique<adas::FCWMonitor>(fcw_config);

  g_ego_frame = std::make_unique<adas::EgoFrame>();
  g_ego_frame->init();

  std::cout << "[Main] Stage E fusion initialized (Replay Mode)\n";

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

  // Stop in reverse order
  if (g_stage_b_manager) {
    g_stage_b_manager->stop();
    g_stage_b_manager.reset();
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
  if (g_stage_b_manager) {
    g_stage_b_manager->printStatus();
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
  std::string model_path =
      "models/yolov5n.engine"; // Use existing 640x640 engine
  bool auto_start = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--hardware-map" && i + 1 < argc) {
      hw_map_path = argv[++i];
    } else if (arg == "--calib-dir" && i + 1 < argc) {
      calib_dir = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model_path = argv[++i];
    } else if (arg == "--auto-start") {
      auto_start = true;
    } else if (arg == "--help") {
      std::cout
          << "Usage: " << argv[0] << " [options]\n"
          << "Options:\n"
          << "  --config <path>        Path to componentConfig.yaml\n"
          << "  --hardware-map <path>  Path to hardware_map.json\n"
          << "  --calib-dir <path>     Path to calibration directory\n"
          << "  --model <path>         Path to YOLOv8 ONNX model\n"
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
      startPipeline(config, hw_map, calib_dir, model_path);
    }

    // Interactive menu loop
    while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
      printMenu();

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
          startPipeline(config, hw_map, calib_dir, model_path);
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
          std::cout << "\n[Config] Reloaded from: " << config_path << "\n";
          std::cout << "[Config] Camera: front=" << config.cameras.width << "x"
                    << config.cameras.height
                    << " | side=" << config.cameras.side_width << "x"
                    << config.cameras.side_height
                    << " | fps=" << config.cameras.target_fps
                    << " | mjpeg=" << (config.cameras.use_mjpeg ? "yes" : "NO")
                    << "\n";

          // Hot-reload: if pipeline is running, restart ingest to apply new
          // camera settings. Stage B and E keep running — only cameras restart.
          if (g_pipeline_running.load() && g_ingest_manager) {
            std::cout << "[Config] Pipeline is running. Restarting camera "
                         "ingest to apply new settings...\n";
            g_ingest_manager->stop();
            g_ingest_manager =
                std::make_unique<adas::IngestManager>(config, hw_map);
            if (g_recorder && g_recorder->isRecording()) {
              // Re-wire recorder after restart
              g_ingest_manager->setRecorder(g_recorder.get());
            }
            g_ingest_manager->start();
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
          startReplayPipeline(file_path, speed, config, hw_map, calib_dir,
                              model_path);
        }
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
