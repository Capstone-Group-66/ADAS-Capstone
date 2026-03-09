// File: src/stage_e/BEVDashboard.cpp
#include "adas/stage_e/BEVDashboard.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace adas {

BEVDashboard::BEVDashboard(BSDReceiver* bsd_receiver, float c_x, float f_x)
    : bsd_receiver_(bsd_receiver), c_x_(c_x), f_x_(f_x) {}

BEVDashboard::~BEVDashboard() {
  stop();
}

void BEVDashboard::start() {
  if (running_.load()) return;
  running_.store(true);
  thread_ = std::thread(&BEVDashboard::renderLoop, this);
}

void BEVDashboard::stop() {
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void BEVDashboard::update(const std::vector<FusedObject>& fused_objects) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_fused_ = fused_objects;
}

void BEVDashboard::renderLoop() {
  const int canvas_width = 800;
  const int canvas_height = 800;
  const float pixels_per_meter = 15.0f;
  const int ego_x = 400;
  const int ego_y = 700;
  const int ego_w = 40;
  const int ego_h = 80;

  cv::namedWindow("ADAS BEVDashboard", cv::WINDOW_AUTOSIZE);

  while (running_.load()) {
    cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(30, 30, 30));

    // Get BSD States safely
    bool left_bsd = false;
    bool right_bsd = false;
    if (bsd_receiver_) {
      left_bsd = bsd_receiver_->getLeftBSDState();
      right_bsd = bsd_receiver_->getRightBSDState();
    }

    // ── Left Blind Spot Zone ──
    std::vector<cv::Point> left_poly = {
        cv::Point(ego_x - ego_w / 2 - 60, ego_y - 20),
        cv::Point(ego_x - ego_w / 2 - 10, ego_y - 20),
        cv::Point(ego_x - ego_w / 2 - 10, ego_y + ego_h + 40),
        cv::Point(ego_x - ego_w / 2 - 60, ego_y + ego_h + 40)};

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
        cv::Point(ego_x + ego_w / 2 + 10, ego_y - 20),
        cv::Point(ego_x + ego_w / 2 + 60, ego_y - 20),
        cv::Point(ego_x + ego_w / 2 + 60, ego_y + ego_h + 40),
        cv::Point(ego_x + ego_w / 2 + 10, ego_y + ego_h + 40)};

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
    cv::putText(canvas, "EGO", cv::Point(ego_x - 15, ego_y + ego_h / 2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // ── Fused Targets (Stage E) ──
    std::vector<FusedObject> current_fused;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_fused = latest_fused_;
    }

    for (const auto& obj : current_fused) {
      if (!obj.has_radar) continue; // Only plot radar-fused objects for true depth

      // Exact pinhole math for X offset
      float z_m = obj.range_m;
      float x_offset_m = z_m * ((obj.centroid_px.x - c_x_) / f_x_);

      int x_canvas = static_cast<int>(ego_x + (x_offset_m * pixels_per_meter));
      int y_canvas = static_cast<int>(ego_y - (z_m * pixels_per_meter));

      // Don't draw if completely out of bounds (save render time)
      if (x_canvas < -100 || x_canvas > canvas_width + 100 ||
          y_canvas < -100 || y_canvas > canvas_height + 100) {
        continue;
      }

      bool is_threat = (obj.ttc_s <= 3.0f);
      cv::Scalar color = is_threat ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0); // Red or Cyan
      int radius = is_threat ? 10 : 6;

      if (is_threat) {
        // Add glow effect for threat
        cv::circle(canvas, cv::Point(x_canvas, y_canvas), radius + 6,
                   cv::Scalar(0, 0, 150), -1);
      }
      cv::circle(canvas, cv::Point(x_canvas, y_canvas), radius, color, -1);

      // Telemetry Text
      std::stringstream ss;
      ss << "[" << (obj.object_id == UINT64_MAX ? 0 : obj.object_id) << "] Z: "
         << std::fixed << std::setprecision(1) << obj.range_m << "m | V: "
         << static_cast<int>(obj.radial_vel_mps) << "m/s";

      int baseline = 0;
      cv::Size text_size = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_SIMPLEX,
                                           0.4, 1, &baseline);
      cv::putText(canvas, ss.str(),
                  cv::Point(x_canvas - text_size.width / 2,
                            y_canvas - radius - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
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
