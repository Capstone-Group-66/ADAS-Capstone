// File: src/stage_e/BEVDashboard.cpp
#include "adas/stage_e/BEVDashboard.hpp"

#include "adas/common/Clock.hpp"
#include "adas/stage_e/BEVDashboardMath.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <unordered_set>

namespace {

constexpr int kCanvasWidth = 300;
constexpr int kCanvasHeight = 300;
constexpr float kPixelsPerMeter = 7.0f;
constexpr int kEgoX = 150;
constexpr int kEgoY = 240;
constexpr int kEgoW = 20;
constexpr int kEgoH = 40;
constexpr float kRadarHalfFovDeg = 10.0f;  // 20 degree cone
constexpr float kCameraHalfFovDeg = 12.5f; // 25 degree corridor span
constexpr float kRadarHeatDecay = 0.94f;
constexpr float kRadarHeatBlurSigma = 2.2f;
constexpr float kRadarHeatOverlayAlpha = 0.55f;
constexpr int kRadarHeatArcThicknessPx = 3;

int toCanvasX(float x_m) {
  return static_cast<int>(kEgoX + x_m * kPixelsPerMeter);
}

int toCanvasY(float z_m) {
  return static_cast<int>(kEgoY - z_m * kPixelsPerMeter);
}

float maxDisplayRangeM() {
  return static_cast<float>(kEgoY - 8) / kPixelsPerMeter;
}

void drawProgressBar(cv::Mat &canvas, int x, int y, int width, int height,
                     float ratio, const cv::Scalar &fg,
                     const cv::Scalar &bg = cv::Scalar(40, 40, 40)) {
  ratio = std::clamp(ratio, 0.0f, 1.0f);
  cv::rectangle(canvas, cv::Rect(x, y, width, height), bg, cv::FILLED);
  cv::rectangle(canvas, cv::Rect(x, y, static_cast<int>(width * ratio), height),
                fg, cv::FILLED);
  cv::rectangle(canvas, cv::Rect(x, y, width, height),
                cv::Scalar(120, 120, 120), 1);
}

void drawRadarCone(cv::Mat &canvas) {
  const float z_far = maxDisplayRangeM();
  const float x_left = adas::bev::lateralFromRangeAndAngle(
      z_far, -adas::bev::degToRad(kRadarHalfFovDeg));
  const float x_right = adas::bev::lateralFromRangeAndAngle(
      z_far, adas::bev::degToRad(kRadarHalfFovDeg));

  cv::line(canvas, cv::Point(kEgoX, kEgoY),
           cv::Point(toCanvasX(x_left), toCanvasY(z_far)),
           cv::Scalar(65, 85, 100), 1);
  cv::line(canvas, cv::Point(kEgoX, kEgoY),
           cv::Point(toCanvasX(x_right), toCanvasY(z_far)),
           cv::Scalar(65, 85, 100), 1);
}

void drawRadarRangeLine(cv::Mat &canvas, float range_m, const cv::Scalar &color,
                        int thickness) {
  if (range_m <= 0.1f || range_m > maxDisplayRangeM()) {
    return;
  }
  const float x_left = adas::bev::lateralFromRangeAndAngle(
      range_m, -adas::bev::degToRad(kRadarHalfFovDeg));
  const float x_right = adas::bev::lateralFromRangeAndAngle(
      range_m, adas::bev::degToRad(kRadarHalfFovDeg));

  cv::line(canvas, cv::Point(toCanvasX(x_left), toCanvasY(range_m)),
           cv::Point(toCanvasX(x_right), toCanvasY(range_m)), color, thickness);
  cv::circle(canvas, cv::Point(kEgoX, toCanvasY(range_m)), 2, color,
             cv::FILLED);
}

void accumulateRadarArc(cv::Mat &accumulator, float range_m, float intensity) {
  if (accumulator.empty() || range_m <= 0.1f || range_m > maxDisplayRangeM()) {
    return;
  }

  const int radius_px = static_cast<int>(std::round(range_m * kPixelsPerMeter));
  if (radius_px <= 0) {
    return;
  }

  const double start_deg = -90.0 - static_cast<double>(kRadarHalfFovDeg);
  const double end_deg = -90.0 + static_cast<double>(kRadarHalfFovDeg);
  cv::ellipse(accumulator, cv::Point(kEgoX, kEgoY),
              cv::Size(radius_px, radius_px), 0.0, start_deg, end_deg,
              cv::Scalar(intensity), kRadarHeatArcThicknessPx, cv::LINE_8);
}

void drawRadarHeatmap(cv::Mat &canvas, cv::Mat &radar_heat_accumulator,
                      const adas::RadarTargets &targets,
                      bool accumulate_raw_radar_tick) {
  if (radar_heat_accumulator.empty() ||
      radar_heat_accumulator.rows != canvas.rows ||
      radar_heat_accumulator.cols != canvas.cols) {
    radar_heat_accumulator = cv::Mat::zeros(canvas.rows, canvas.cols, CV_32FC1);
  }

  radar_heat_accumulator *= kRadarHeatDecay;

  if (accumulate_raw_radar_tick) {
    for (const auto &target : targets.targets) {
      accumulateRadarArc(radar_heat_accumulator, target.range_m, 1.0f);
    }
  }

  cv::Mat blurred;
  cv::GaussianBlur(radar_heat_accumulator, blurred, cv::Size(0, 0),
                   kRadarHeatBlurSigma);

  double max_value = 0.0;
  cv::minMaxLoc(blurred, nullptr, &max_value);
  if (max_value <= 1e-6) {
    return;
  }

  cv::Mat heat_u8;
  const double normalize_scale = 255.0 / std::max(1.0, max_value);
  blurred.convertTo(heat_u8, CV_8UC1, normalize_scale);

  cv::Mat mask;
  cv::threshold(heat_u8, mask, 2, 255, cv::THRESH_BINARY);
  if (cv::countNonZero(mask) == 0) {
    return;
  }

  cv::Mat heat_color;
  cv::applyColorMap(heat_u8, heat_color, cv::COLORMAP_JET);

  cv::Mat blended;
  cv::addWeighted(canvas, 1.0, heat_color, kRadarHeatOverlayAlpha, 0.0,
                  blended);
  blended.copyTo(canvas, mask);
}

void drawCorridor(cv::Mat &canvas, float angle_rad, const cv::Scalar &color,
                  int thickness) {
  const float z_far = maxDisplayRangeM();
  const float x_far = adas::bev::lateralFromRangeAndAngle(z_far, angle_rad);
  cv::line(canvas, cv::Point(kEgoX, kEgoY),
           cv::Point(toCanvasX(x_far), toCanvasY(z_far)), color, thickness,
           cv::LINE_AA);
}

void drawCrosshair(cv::Mat &canvas, int x, int y, const cv::Scalar &color,
                   int size_px = 5, int thickness = 2) {
  cv::line(canvas, cv::Point(x - size_px, y), cv::Point(x + size_px, y), color,
           thickness);
  cv::line(canvas, cv::Point(x, y - size_px), cv::Point(x, y + size_px), color,
           thickness);
}

void drawBlindSpotZones(cv::Mat &canvas, bool left_bsd, bool right_bsd) {
  std::vector<cv::Point> left_poly = {
      cv::Point(kEgoX - kEgoW / 2 - 2, kEgoY + 15),
      cv::Point(kEgoX - kEgoW / 2 - 2, kEgoY + kEgoH + 5),
      cv::Point(kEgoX - kEgoW / 2 - 35, kEgoY + kEgoH + 25),
      cv::Point(kEgoX - kEgoW / 2 - 30, kEgoY + 10)};

  if (left_bsd) {
    cv::Mat overlay;
    canvas.copyTo(overlay);
    cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{left_poly},
                 cv::Scalar(0, 0, 255));
    cv::addWeighted(overlay, 0.6, canvas, 0.4, 0, canvas);
  } else {
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{left_poly}, true,
                  cv::Scalar(100, 100, 100), 2);
  }

  std::vector<cv::Point> right_poly = {
      cv::Point(kEgoX + kEgoW / 2 + 2, kEgoY + 15),
      cv::Point(kEgoX + kEgoW / 2 + 2, kEgoY + kEgoH + 5),
      cv::Point(kEgoX + kEgoW / 2 + 35, kEgoY + kEgoH + 25),
      cv::Point(kEgoX + kEgoW / 2 + 30, kEgoY + 10)};

  if (right_bsd) {
    cv::Mat overlay;
    canvas.copyTo(overlay);
    cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{right_poly},
                 cv::Scalar(0, 0, 255));
    cv::addWeighted(overlay, 0.6, canvas, 0.4, 0, canvas);
  } else {
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{right_poly}, true,
                  cv::Scalar(100, 100, 100), 2);
  }
}

void drawEgoVehicle(cv::Mat &canvas) {
  cv::rectangle(canvas, cv::Rect(kEgoX - kEgoW / 2, kEgoY, kEgoW, kEgoH),
                cv::Scalar(200, 200, 200), cv::FILLED);
  cv::putText(canvas, "EGO", cv::Point(kEgoX - 12, kEgoY + kEgoH / 2 + 4),
              cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);
}

} // namespace

namespace adas {

BEVDashboard::BEVDashboard(BSDReceiver *bsd_receiver, float c_x, float f_x,
                           uint32_t dead_track_cleanup_ms, uint32_t ttc_hold_ms)
    : bsd_receiver_(bsd_receiver), c_x_(c_x), f_x_(f_x),
      dead_track_cleanup_ms_(dead_track_cleanup_ms), ttc_hold_ms_(ttc_hold_ms) {
}

BEVDashboard::~BEVDashboard() { stop(); }

void BEVDashboard::start() {
  if (running_.load()) {
    return;
  }
  running_.store(true);
  thread_ = std::thread(&BEVDashboard::renderLoop, this);
}

void BEVDashboard::stop() {
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void BEVDashboard::update(const BEVInputFrame &frame) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_frame_ = frame;
  ++latest_frame_seq_;
}

void BEVDashboard::applyFrameUpdate(const BEVInputFrame &frame) {
  const uint64_t now_ns = frame.now_ns == 0 ? Clock::now_ns() : frame.now_ns;
  std::unordered_set<uint64_t> touched_by_camera;

  for (const auto &det : frame.camera_batch.dets) {
    if (det.object_id == UINT64_MAX) {
      continue;
    }

    const float angle_rad =
        adas::bev::angleFromPixel(det.centroid.x, c_x_, f_x_);
    if (!adas::bev::inAngleSpan(angle_rad, kCameraHalfFovDeg)) {
      continue;
    }

    Track &track = tracks_[det.object_id];
    track.object_id = det.object_id;
    track.corridor_angle_rad = angle_rad;
    track.last_cam_update_ns = now_ns;
    track.has_crosshair = false;

    const int conf_pct = static_cast<int>(std::round(det.score * 100.0f));
    std::stringstream ss;
    ss << "[" << det.object_id << "] C:" << std::clamp(conf_pct, 0, 100) << "%";
    track.label = ss.str();

    touched_by_camera.insert(det.object_id);
  }

  for (const auto &obj : frame.fused_objects) {
    if (obj.object_id == UINT64_MAX) {
      continue;
    }

    Track &track = tracks_[obj.object_id];
    track.object_id = obj.object_id;

    const float angle_rad =
        adas::bev::angleFromPixel(obj.centroid_px.x, c_x_, f_x_);
    if (adas::bev::inAngleSpan(angle_rad, kCameraHalfFovDeg)) {
      track.corridor_angle_rad = angle_rad;
    }

    track.last_cam_update_ns = now_ns;
    track.speed_fresh = obj.speed_fresh;
    track.speed_age_ms = obj.speed_age_ms;
    track.radial_vel_mps = obj.radial_vel_mps;

    if (obj.has_radar && obj.range_m > 0.1f) {
      track.last_range_update_ns = now_ns;
      track.z_m = obj.range_m;
      // Keep existing lateral projection math unchanged.
      track.x_offset_m = obj.range_m * ((obj.centroid_px.x - c_x_) / f_x_);
      track.has_crosshair = true;
    }

    std::stringstream ss;
    ss << "[" << obj.object_id << "] Z:" << std::fixed << std::setprecision(1)
       << obj.range_m << "m";
    if (track.speed_fresh) {
      ss << " V:" << static_cast<int>(std::round(obj.radial_vel_mps)) << "m/s";
    } else {
      ss << " V:stale";
    }
    track.label = ss.str();

    touched_by_camera.insert(obj.object_id);
  }

  if (frame.fcw_focus_object_id.has_value()) {
    auto it = tracks_.find(*frame.fcw_focus_object_id);
    if (it != tracks_.end()) {
      const uint64_t hold_ns = static_cast<uint64_t>(ttc_hold_ms_) * 1000000ULL;
      it->second.ttc_hold_until_ns =
          std::max(it->second.ttc_hold_until_ns, now_ns + hold_ns);
    }
  }

  // Any camera-touched track not fused this frame should stay as ghost
  // corridor.
  for (uint64_t id : touched_by_camera) {
    auto it = tracks_.find(id);
    if (it != tracks_.end() && it->second.last_range_update_ns != now_ns) {
      it->second.has_crosshair = false;
    }
  }
}

void BEVDashboard::renderLoop() {
  cv::namedWindow("ADAS BEVDashboard", cv::WINDOW_AUTOSIZE);
  cv::Mat radar_heat_accumulator =
      cv::Mat::zeros(kCanvasHeight, kCanvasWidth, CV_32FC1);

  uint64_t last_processed_seq = 0;

  while (running_.load()) {
    cv::Mat canvas(kCanvasHeight, kCanvasWidth, CV_8UC3,
                   cv::Scalar(28, 28, 28));

    BEVInputFrame frame;
    uint64_t frame_seq = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      frame = latest_frame_;
      frame_seq = latest_frame_seq_;
    }

    const bool has_new_frame = frame_seq != last_processed_seq;
    if (has_new_frame) {
      applyFrameUpdate(frame);
      last_processed_seq = frame_seq;
    }

    const uint64_t now_ns = Clock::now_ns();

    bool left_bsd = false;
    bool right_bsd = false;
    if (bsd_receiver_) {
      left_bsd = bsd_receiver_->getLeftBSDState();
      right_bsd = bsd_receiver_->getRightBSDState();
    }

    drawBlindSpotZones(canvas, left_bsd, right_bsd);
    drawEgoVehicle(canvas);
    drawRadarCone(canvas);
    drawRadarHeatmap(canvas, radar_heat_accumulator, frame.radar_targets,
                     has_new_frame);

    // Raw radar scan-lines (20 degree cone)
    for (const auto &target : frame.radar_targets.targets) {
      drawRadarRangeLine(canvas, target.range_m, cv::Scalar(190, 110, 30), 1);
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
      Track &track = it->second;

      if (!adas::bev::shouldKeepTrack(
              now_ns, track.last_cam_update_ns, track.last_range_update_ns,
              track.ttc_hold_until_ns, dead_track_cleanup_ms_)) {
        it = tracks_.erase(it);
        continue;
      }

      const float cam_remain = adas::bev::ttlRemaining01(
          now_ns, track.last_cam_update_ns, dead_track_cleanup_ms_);
      const float range_remain = adas::bev::ttlRemaining01(
          now_ns, track.last_range_update_ns, dead_track_cleanup_ms_);
      const bool cam_alive = cam_remain > 0.0f;
      const bool range_alive = range_remain > 0.0f;

      const float hold_remain =
          track.ttc_hold_until_ns > now_ns
              ? static_cast<float>(track.ttc_hold_until_ns - now_ns) /
                    static_cast<float>(ttc_hold_ms_) / 1000000.0f
              : 0.0f;

      const bool hold_active = hold_remain > 0.0f;
      const bool crosshair_active =
          cam_alive && range_alive && track.has_crosshair;

      // Camera corridor layer (25 degree span)
      if (cam_alive &&
          adas::bev::inAngleSpan(track.corridor_angle_rad, kCameraHalfFovDeg)) {
        const cv::Scalar corridor_color =
            crosshair_active ? cv::Scalar(120, 220, 120)
                             : cv::Scalar(90, 90, 170); // ghost
        drawCorridor(canvas, track.corridor_angle_rad, corridor_color,
                     crosshair_active ? 2 : 1);
      }

      int anchor_x = kEgoX;
      int anchor_y = kEgoY - 40;

      if (crosshair_active && track.z_m > 0.1f) {
        anchor_x = toCanvasX(track.x_offset_m);
        anchor_y = toCanvasY(track.z_m);

        if (anchor_x >= 0 && anchor_x < kCanvasWidth && anchor_y >= 0 &&
            anchor_y < kCanvasHeight) {
          const cv::Scalar marker_color =
              hold_active ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 255);
          drawCrosshair(canvas, anchor_x, anchor_y, marker_color, 6, 2);

          if (hold_active) {
            cv::circle(canvas, cv::Point(anchor_x, anchor_y), 10,
                       cv::Scalar(0, 0, 180), 2);
          }

          if (track.speed_fresh && std::abs(track.radial_vel_mps) > 0.1f) {
            const int arrow_len = 12;
            const int dir = (track.radial_vel_mps < 0.0f) ? 1 : -1;
            cv::arrowedLine(canvas, cv::Point(anchor_x, anchor_y),
                            cv::Point(anchor_x, anchor_y + dir * arrow_len),
                            cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.35);
          }
        }
      } else if (range_alive && !cam_alive && track.z_m > 0.1f) {
        // Range-only track visualization fallback.
        anchor_x = toCanvasX(track.x_offset_m);
        anchor_y = toCanvasY(track.z_m);
        cv::circle(canvas, cv::Point(anchor_x, anchor_y), 4,
                   cv::Scalar(200, 110, 30), cv::FILLED);
      }

      // TTL bars: stale cleanup + TTC hold timer.
      drawProgressBar(canvas, anchor_x - 18, anchor_y - 18, 36, 4,
                      std::max(cam_remain, range_remain),
                      cv::Scalar(0, 180, 230));
      if (hold_active) {
        drawProgressBar(canvas, anchor_x - 18, anchor_y - 24, 36, 4,
                        std::clamp(hold_remain, 0.0f, 1.0f),
                        cv::Scalar(0, 0, 255));
      }

      // Text label and speed freshness note.
      const cv::Scalar text_color = track.speed_fresh
                                        ? cv::Scalar(230, 230, 230)
                                        : cv::Scalar(160, 160, 160);
      cv::putText(canvas, track.label, cv::Point(anchor_x - 40, anchor_y - 28),
                  cv::FONT_HERSHEY_SIMPLEX, 0.32, text_color, 1);

      ++it;
    }

    cv::imshow("ADAS BEVDashboard", canvas);
    cv::waitKey(33);
  }

  try {
    cv::destroyWindow("ADAS BEVDashboard");
  } catch (...) {
  }
}

} // namespace adas
