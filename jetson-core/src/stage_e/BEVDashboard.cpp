// File: src/stage_e/BEVDashboard.cpp
#include "adas/stage_e/BEVDashboard.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

namespace adas {

BEVDashboard::BEVDashboard(BSDReceiver *bsd_receiver, float c_x, float f_x)
    : bsd_receiver_(bsd_receiver), c_x_(c_x), f_x_(f_x) {}

BEVDashboard::~BEVDashboard() { stop(); }

void BEVDashboard::start() {
  if (running_.load())
    return;
  running_.store(true);
  thread_ = std::thread(&BEVDashboard::renderLoop, this);
}

void BEVDashboard::stop() {
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void BEVDashboard::update(const std::vector<FusedObject> &fused_objects) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_fused_ = fused_objects;
}

void BEVDashboard::renderLoop() {
  const int canvas_width = 300;
  const int canvas_height = 300;
  const float pixels_per_meter = 7.0f;
  const int ego_x = 150;
  const int ego_y = 240;
  const int ego_w = 20;
  const int ego_h = 40;

  cv::namedWindow("ADAS BEVDashboard", cv::WINDOW_AUTOSIZE);

  while (running_.load()) {
    cv::Mat canvas(canvas_height, canvas_width, CV_8UC3,
                   cv::Scalar(30, 30, 30));

    // Get BSD States safely
    bool left_bsd = false;
    bool right_bsd = false;
    if (bsd_receiver_) {
      left_bsd = bsd_receiver_->getLeftBSDState();
      right_bsd = bsd_receiver_->getRightBSDState();
    }

    // ── Left Blind Spot Zone ──
    std::vector<cv::Point> left_poly = {
        cv::Point(ego_x - ego_w / 2 - 2, ego_y + 15),         // Near side mirror
        cv::Point(ego_x - ego_w / 2 - 2, ego_y + ego_h + 5),  // Near rear bumper
        cv::Point(ego_x - ego_w / 2 - 35, ego_y + ego_h + 25), // Wide left, extending behind
        cv::Point(ego_x - ego_w / 2 - 30, ego_y + 10)         // Wide left, near mirror front
    };

    if (left_bsd) {
      cv::Mat overlay;
      canvas.copyTo(overlay);
      cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{left_poly},
                   cv::Scalar(0, 0, 255));
      cv::addWeighted(overlay, 0.6, canvas, 0.4, 0, canvas);
    } else {
      cv::polylines(canvas, std::vector<std::vector<cv::Point>>{left_poly},
                    true, cv::Scalar(100, 100, 100), 2);
    }

    // ── Right Blind Spot Zone ──
    std::vector<cv::Point> right_poly = {
        cv::Point(ego_x + ego_w / 2 + 2, ego_y + 15),         // Near side mirror
        cv::Point(ego_x + ego_w / 2 + 2, ego_y + ego_h + 5),  // Near rear bumper
        cv::Point(ego_x + ego_w / 2 + 35, ego_y + ego_h + 25), // Wide right, extending behind
        cv::Point(ego_x + ego_w / 2 + 30, ego_y + 10)         // Wide right, near mirror front
    };

    if (right_bsd) {
      cv::Mat overlay;
      canvas.copyTo(overlay);
      cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{right_poly},
                   cv::Scalar(0, 0, 255));
      cv::addWeighted(overlay, 0.6, canvas, 0.4, 0, canvas);
    } else {
      cv::polylines(canvas, std::vector<std::vector<cv::Point>>{right_poly},
                    true, cv::Scalar(100, 100, 100), 2);
    }

    // ── Ego Vehicle ──
    cv::rectangle(canvas, cv::Rect(ego_x - ego_w / 2, ego_y, ego_w, ego_h),
                  cv::Scalar(200, 200, 200), -1);
    cv::putText(canvas, "EGO", cv::Point(ego_x - 12, ego_y + ego_h / 2 + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);

    // ── Fused Targets (Stage E) ──
    std::vector<FusedObject> current_fused;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_fused = latest_fused_;
    }

    auto now = std::chrono::steady_clock::now();

    // 1. Update tracks with current detections
    for (const auto &obj : current_fused) {
      if (!obj.has_radar) continue; // Only plot radar-fused objects for true depth

      float z_m = obj.range_m;
      float x_offset_m = z_m * ((obj.centroid_px.x - c_x_) / f_x_);
      bool is_threat = (obj.ttc_s <= 3.0f);

      std::stringstream ss;
      ss << "[" << (obj.object_id == UINT64_MAX ? 0 : obj.object_id)
         << "] Z: " << std::fixed << std::setprecision(1) << obj.range_m
         << "m | V: " << static_cast<int>(obj.radial_vel_mps) << "m/s";

      auto it = tracks_.find(obj.object_id);
      if (it != tracks_.end()) {
        it->second.x_offset_m = x_offset_m;
        it->second.z_m = z_m;
        // Keep threat status active to force 5s hold if it was a threat before,
        // or upgrade to threat if newly a threat.
        if (is_threat) it->second.is_threat = true; 
        it->second.label = ss.str();
        it->second.radial_vel_mps = obj.radial_vel_mps;
        it->second.last_seen = now;
      } else {
        Track t = {x_offset_m, z_m, is_threat, ss.str(), obj.radial_vel_mps, now};
        tracks_[obj.object_id] = t;
      }
    }

    // 2. Render all internal tracks and fade them out based on age
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
      float age_s = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now - it->second.last_seen).count() / 1000.0f;
                      
      float max_age_s = it->second.is_threat ? 5.0f : 1.5f; // Threat holds 5s, normal holds 1.5s

      if (age_s > max_age_s) {
        it = tracks_.erase(it);
        continue;
      }

      // Calculate fade (alpha) where 1.0 is fully opaque, 0.0 is transparent
      // Fade out more aggressively in the last half of its lifespan
      float alpha = 1.0f - (age_s / max_age_s);
      alpha = std::max(0.0f, std::min(1.0f, alpha));

      int x_canvas = static_cast<int>(ego_x + (it->second.x_offset_m * pixels_per_meter));
      int y_canvas = static_cast<int>(ego_y - (it->second.z_m * pixels_per_meter));

      if (x_canvas < -100 || x_canvas > canvas_width + 100 || 
          y_canvas < -100 || y_canvas > canvas_height + 100) {
        ++it;
        continue;
      }

      int radius = it->second.is_threat ? 6 : 4;
      cv::Scalar base_color = it->second.is_threat ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0); // Red or Cyan
      cv::Scalar color = base_color * alpha; // Multiply by alpha to get darker/faded color

      if (it->second.is_threat) {
        // Warning glow ring
        cv::Scalar glow_color = cv::Scalar(0, 0, 150) * alpha;
        cv::circle(canvas, cv::Point(x_canvas, y_canvas), radius + 4, glow_color, -1);
      }
      cv::circle(canvas, cv::Point(x_canvas, y_canvas), radius, color, -1);

      // Telemetry Text (also fading)
      int baseline = 0;
      cv::Size text_size = cv::getTextSize(it->second.label, cv::FONT_HERSHEY_SIMPLEX, 0.3, 1, &baseline);
      cv::putText(
          canvas, it->second.label,
          cv::Point(x_canvas - text_size.width / 2, y_canvas - radius - 3),
          cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255 * alpha, 255 * alpha, 255 * alpha), 1);

      ++it;
    }

    cv::imshow("ADAS BEVDashboard", canvas);

    // CRITICAL FIX: Pump X11 event queue and maintain ~30Hz render loop
    cv::waitKey(33);
  }

  try {
    cv::destroyWindow("ADAS BEVDashboard");
  } catch (...) {
  }
}

} // namespace adas
