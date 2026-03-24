// File: src/stage_e/BEVDashboard.cpp
#include "adas/stage_e/BEVDashboard.hpp"

#include "adas/common/Clock.hpp"
#include "adas/stage_e/BEVDashboardFlow.hpp"
#include "adas/stage_e/BEVDashboardMath.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <utility>

namespace {

constexpr int kCanvasWidth = 520;
constexpr int kCanvasHeight = 660;
constexpr int kPanelWidth = 620;
constexpr int kPanelHeight = kCanvasHeight;
constexpr int kDisplayMaxWidth = 760;
constexpr int kDisplayMaxHeight = 420;
constexpr uint64_t kPanelSnapshotHoldNs = 3000000000ULL;
constexpr uint64_t kPanelSnapshotMinRefreshNs = 450000000ULL;
constexpr float kPixelsPerMeter = 10.0f;
constexpr int kEgoX = kCanvasWidth / 2;
constexpr int kEgoY = 560;
constexpr int kEgoW = 28;
constexpr int kEgoH = 52;
constexpr float kRadarHalfFovDeg = 10.0f;  // 20 degree cone
constexpr float kCameraHalfFovDeg = 12.5f; // 25 degree corridor span
constexpr float kRadarRangeTxM = 0.0127f;  // Keep same t_x used in fusion.
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

float adjustedRadarRangeM(float raw_range_m) {
  const float adjusted = raw_range_m - kRadarRangeTxM;
  return adjusted > 0.0f ? adjusted : 0.0f;
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
  const float adjusted_range_m = adjustedRadarRangeM(range_m);
  if (adjusted_range_m <= 0.1f || adjusted_range_m > maxDisplayRangeM()) {
    return;
  }
  const float x_left = adas::bev::lateralFromRangeAndAngle(
      adjusted_range_m, -adas::bev::degToRad(kRadarHalfFovDeg));
  const float x_right = adas::bev::lateralFromRangeAndAngle(
      adjusted_range_m, adas::bev::degToRad(kRadarHalfFovDeg));

  cv::line(canvas, cv::Point(toCanvasX(x_left), toCanvasY(adjusted_range_m)),
           cv::Point(toCanvasX(x_right), toCanvasY(adjusted_range_m)), color,
           thickness);
  cv::circle(canvas, cv::Point(kEgoX, toCanvasY(adjusted_range_m)), 2, color,
             cv::FILLED);
}

void accumulateRadarArc(cv::Mat &accumulator, float range_m, float intensity) {
  const float adjusted_range_m = adjustedRadarRangeM(range_m);
  if (accumulator.empty() || adjusted_range_m <= 0.1f ||
      adjusted_range_m > maxDisplayRangeM()) {
    return;
  }

  // Draw range band in longitudinal-Z space to match fused target placement.
  const float x_left = adas::bev::lateralFromRangeAndAngle(
      adjusted_range_m, -adas::bev::degToRad(kRadarHalfFovDeg));
  const float x_right = adas::bev::lateralFromRangeAndAngle(
      adjusted_range_m, adas::bev::degToRad(kRadarHalfFovDeg));
  const int y_px = toCanvasY(adjusted_range_m);
  cv::line(accumulator, cv::Point(toCanvasX(x_left), y_px),
           cv::Point(toCanvasX(x_right), y_px), cv::Scalar(intensity),
           kRadarHeatArcThicknessPx, cv::LINE_8);
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

void drawFcwDirectionRay(cv::Mat &canvas, const cv::Point &target) {
  const cv::Point ego(kEgoX, kEgoY);
  cv::line(canvas, ego, target, cv::Scalar(0, 0, 70), 12, cv::LINE_AA);
  cv::line(canvas, ego, target, cv::Scalar(0, 0, 140), 8, cv::LINE_AA);
  cv::line(canvas, ego, target, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
  cv::circle(canvas, target, 5, cv::Scalar(0, 0, 255), cv::FILLED);
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

cv::Scalar fcwLevelColor(uint8_t level) {
  switch (level) {
  case 3:
    return cv::Scalar(30, 30, 220);
  case 2:
    return cv::Scalar(0, 140, 255);
  case 1:
    return cv::Scalar(0, 215, 235);
  default:
    return cv::Scalar(130, 130, 130);
  }
}

std::string fmtFloat(float value, const char *suffix, int precision = 2) {
  if (!std::isfinite(value)) {
    return std::string("--");
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << value << suffix;
  return ss.str();
}

bool snapshotHasDisplayContent(const std::optional<adas::FCWDebugSnapshot> &debug_opt) {
  return debug_opt.has_value() &&
         (debug_opt->has_best_candidate ||
          debug_opt->has_runner_up_candidate ||
          !debug_opt->rejected_candidates.empty());
}

uint64_t snapshotBestId(const adas::FCWDebugSnapshot &snapshot) {
  return snapshot.has_best_candidate ? snapshot.best_candidate.object_id
                                     : UINT64_MAX;
}

bool sourceFresh(uint64_t now_ns, uint64_t timestamp_ns,
                 uint64_t fresh_window_ns = 1100000000ULL) {
  return timestamp_ns != 0 && now_ns <= timestamp_ns + fresh_window_ns;
}

cv::Scalar dimColor(const cv::Scalar &color, double factor) {
  factor = std::clamp(factor, 0.0, 1.0);
  return cv::Scalar(color[0] * factor, color[1] * factor, color[2] * factor);
}

std::string ageLabel(uint64_t now_ns, uint64_t timestamp_ns) {
  if (timestamp_ns == 0) {
    return "--";
  }
  if (now_ns <= timestamp_ns) {
    return "0ms";
  }

  const uint64_t age_ms = (now_ns - timestamp_ns) / 1000000ULL;
  if (age_ms < 1000ULL) {
    return std::to_string(age_ms) + "ms";
  }

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1)
     << (static_cast<double>(age_ms) / 1000.0) << "s";
  return ss.str();
}

cv::Point pointOnSegment(const cv::Point &a, const cv::Point &b, float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  const float x = static_cast<float>(a.x) +
                  (static_cast<float>(b.x) - static_cast<float>(a.x)) * t;
  const float y = static_cast<float>(a.y) +
                  (static_cast<float>(b.y) - static_cast<float>(a.y)) * t;
  return cv::Point(static_cast<int>(std::lround(x)),
                   static_cast<int>(std::lround(y)));
}

cv::Point pointOnPolyline(const std::vector<cv::Point> &points, float t) {
  if (points.empty()) {
    return cv::Point();
  }
  if (points.size() == 1) {
    return points.front();
  }

  t = std::clamp(t, 0.0f, 1.0f);
  std::vector<double> lengths;
  lengths.reserve(points.size() - 1);
  double total_length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const double dx = static_cast<double>(points[i].x - points[i - 1].x);
    const double dy = static_cast<double>(points[i].y - points[i - 1].y);
    const double segment = std::sqrt(dx * dx + dy * dy);
    lengths.push_back(segment);
    total_length += segment;
  }

  if (total_length <= 1e-6) {
    return points.back();
  }

  double target = total_length * static_cast<double>(t);
  for (size_t i = 0; i < lengths.size(); ++i) {
    if (target <= lengths[i] || i + 1 == lengths.size()) {
      const float local_t =
          lengths[i] <= 1e-6 ? 0.0f : static_cast<float>(target / lengths[i]);
      return pointOnSegment(points[i], points[i + 1], local_t);
    }
    target -= lengths[i];
  }
  return points.back();
}

void drawPulsedPolyline(cv::Mat &canvas, const std::vector<cv::Point> &points,
                        const cv::Scalar &color, int thickness,
                        uint64_t now_ns, bool held, double dim_factor = 0.45) {
  if (points.size() < 2) {
    return;
  }

  for (size_t i = 1; i < points.size(); ++i) {
    cv::line(canvas, points[i - 1], points[i], dimColor(color, dim_factor),
             thickness, cv::LINE_AA);
  }

  const double seconds = static_cast<double>(now_ns) / 1.0e9;
  const double speed = held ? 0.24 : 0.48;
  const double phase = std::fmod(seconds * speed, 1.0);
  constexpr std::array<float, 3> kOffsets = {0.0f, 0.28f, 0.56f};
  for (const float offset : kOffsets) {
    const float pulse_t = static_cast<float>(std::fmod(phase + offset, 1.0));
    const cv::Point pulse = pointOnPolyline(points, pulse_t);
    cv::circle(canvas, pulse, thickness + 2, color, cv::FILLED, cv::LINE_AA);
    cv::circle(canvas, pulse, std::max(2, thickness - 1),
               cv::Scalar(245, 245, 245), cv::FILLED, cv::LINE_AA);
  }
}

cv::Point flowStagePoint(adas::bev::FlowStage stage) {
  switch (stage) {
  case adas::bev::FlowStage::Fusion:
    return cv::Point(kPanelWidth / 2, 176);
  case adas::bev::FlowStage::CameraFresh:
    return cv::Point(186, 262);
  case adas::bev::FlowStage::HasRadar:
    return cv::Point(434, 262);
  case adas::bev::FlowStage::SpeedFresh:
    return cv::Point(186, 292);
  case adas::bev::FlowStage::ClassRelevant:
    return cv::Point(434, 292);
  case adas::bev::FlowStage::RangeOk:
    return cv::Point(186, 322);
  case adas::bev::FlowStage::QualityOk:
    return cv::Point(434, 322);
  case adas::bev::FlowStage::ClosingOk:
    return cv::Point(186, 352);
  case adas::bev::FlowStage::InPath:
    return cv::Point(434, 352);
  case adas::bev::FlowStage::RiskMix:
    return cv::Point(kPanelWidth / 2, 448);
  case adas::bev::FlowStage::StateMachine:
    return cv::Point(kPanelWidth / 2, 556);
  case adas::bev::FlowStage::Output:
  default:
    return cv::Point(kPanelWidth / 2, 621);
  }
}

void renderFcwThoughtFlowPanel(
    cv::Mat &panel, const std::optional<adas::FCWDebugSnapshot> &debug_opt,
    bool snapshot_is_held, float hold_remaining_s, int camera_count,
    uint64_t camera_ts_ns, bool camera_healthy, int radar_count,
    uint64_t radar_ts_ns, bool radar_healthy,
    const std::optional<adas::EgoDebugSnapshot> &ego_debug_opt, int fused_count,
    bool ble_send_active, uint64_t now_ns) {
  panel = cv::Mat(kPanelHeight, kPanelWidth, CV_8UC3, cv::Scalar(24, 24, 26));

  const auto drawPanelText = [&](const std::string &text, int x, int y,
                                 double scale, const cv::Scalar &color,
                                 int thickness = 1) {
    cv::putText(panel, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv::LINE_AA);
  };

  const auto drawBadge =
      [&](int x, int y, const std::string &label, bool active,
          const cv::Scalar &color) -> int {
    int baseline = 0;
    const cv::Size text_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.31, 1, &baseline);
    const int pad_x = 7;
    const int width = text_size.width + pad_x * 2;
    const int height = 17;
    cv::rectangle(panel, cv::Rect(x, y, width, height),
                  active ? color : cv::Scalar(52, 52, 56), cv::FILLED);
    cv::rectangle(panel, cv::Rect(x, y, width, height),
                  active ? color : cv::Scalar(92, 92, 96), 1);
    drawPanelText(label, x + pad_x, y + 12, 0.31,
                  active ? cv::Scalar(245, 245, 245)
                         : cv::Scalar(150, 150, 155),
                  1);
    return width + 6;
  };

  const auto drawBar =
      [&](int x, int y, int width, const std::string &label, float value,
          float weight, const cv::Scalar &color) {
    const float clamped = std::clamp(value, 0.0f, 1.0f);
    cv::rectangle(panel, cv::Rect(x, y, width, 10), cv::Scalar(42, 42, 46),
                  cv::FILLED);
    cv::rectangle(panel, cv::Rect(x, y, width, 10), cv::Scalar(90, 90, 96), 1);
    cv::rectangle(panel, cv::Rect(x, y, static_cast<int>(width * clamped), 10),
                  color, cv::FILLED);

    std::ostringstream rhs;
    rhs << std::fixed << std::setprecision(2) << value << " x" << weight
        << " = " << (value * weight);
    drawPanelText(label, x, y - 4, 0.30, cv::Scalar(205, 205, 210), 1);
    drawPanelText(rhs.str(), x + width - 160, y - 4, 0.29,
                  cv::Scalar(175, 175, 180), 1);
  };

  const auto drawCard = [&](const std::string &title, const std::string &line1,
                            const std::string &line2, const cv::Rect &rect,
                            bool live, const cv::Scalar &accent) {
    cv::rectangle(panel, rect, cv::Scalar(31, 31, 35), cv::FILLED);
    cv::rectangle(panel, rect, live ? accent : cv::Scalar(82, 82, 86),
                  live ? 2 : 1);
    cv::circle(panel, cv::Point(rect.x + rect.width - 14, rect.y + 14), 5,
               live ? accent : cv::Scalar(60, 60, 70), cv::FILLED,
               cv::LINE_AA);
    drawPanelText(title, rect.x + 10, rect.y + 18, 0.34,
                  cv::Scalar(235, 235, 240), 1);
    drawPanelText(line1, rect.x + 10, rect.y + 40, 0.32,
                  cv::Scalar(220, 220, 225), 1);
    drawPanelText(line2, rect.x + 10, rect.y + 58, 0.29,
                  live ? cv::Scalar(185, 205, 225)
                       : cv::Scalar(145, 145, 150),
                  1);
  };

  const auto drawStageBox = [&](const std::string &title, const cv::Rect &rect,
                                const cv::Scalar &edge) {
    cv::rectangle(panel, rect, cv::Scalar(30, 30, 34), cv::FILLED);
    cv::rectangle(panel, rect, edge, 1);
    cv::putText(panel, title, cv::Point(rect.x + 10, rect.y + 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(235, 235, 240), 1,
                cv::LINE_AA);
  };

  const auto drawStageChip = [&](adas::bev::FlowStage stage,
                                 const std::string &label, bool active,
                                 const cv::Scalar &accent) {
    const cv::Point center = flowStagePoint(stage);
    const cv::Rect chip(center.x - 56, center.y - 11, 112, 22);
    const cv::Scalar edge = active ? accent : cv::Scalar(72, 72, 78);
    cv::rectangle(panel, chip, cv::Scalar(34, 34, 38), cv::FILLED);
    cv::rectangle(panel, chip, edge, active ? 2 : 1);
    if (active) {
      cv::circle(panel, cv::Point(chip.x + 10, chip.y + chip.height / 2), 4,
                 accent, cv::FILLED, cv::LINE_AA);
    }
    drawPanelText(label, chip.x + 20, chip.y + 15, 0.31,
                  active ? cv::Scalar(240, 240, 244)
                         : cv::Scalar(155, 155, 160),
                  1);
  };

  cv::rectangle(panel, cv::Rect(10, 10, kPanelWidth - 20, 54),
                cv::Scalar(31, 31, 35), cv::FILLED);
  cv::rectangle(panel, cv::Rect(10, 10, kPanelWidth - 20, 54),
                cv::Scalar(74, 74, 80), 1);
  drawPanelText("FCW Thought Flow", 20, 30, 0.52,
                cv::Scalar(245, 245, 248), 1);
  drawPanelText(snapshot_is_held ? "HELD" : "LIVE", kPanelWidth - 84, 30,
                0.38,
                snapshot_is_held ? cv::Scalar(255, 220, 140)
                                 : cv::Scalar(110, 235, 180),
                1);

  const bool camera_live = camera_healthy && sourceFresh(now_ns, camera_ts_ns);
  const bool radar_live = radar_healthy && sourceFresh(now_ns, radar_ts_ns);
  const bool ego_live = ego_debug_opt.has_value() && ego_debug_opt->valid &&
                        sourceFresh(now_ns, ego_debug_opt->timestamp_ns);

  std::ostringstream cam_line1;
  cam_line1 << camera_count << " detections";
  std::ostringstream cam_line2;
  cam_line2 << "fresh " << ageLabel(now_ns, camera_ts_ns);
  drawCard("CAMERA", cam_line1.str(), cam_line2.str(),
           cv::Rect(18, 76, 182, 68), camera_live,
           cv::Scalar(255, 190, 90));

  std::ostringstream radar_line1;
  radar_line1 << radar_count << " targets";
  std::ostringstream radar_line2;
  radar_line2 << "fresh " << ageLabel(now_ns, radar_ts_ns);
  drawCard("RADAR", radar_line1.str(), radar_line2.str(),
           cv::Rect(219, 76, 182, 68), radar_live,
           cv::Scalar(80, 180, 255));

  std::ostringstream ego_line1;
  if (ego_debug_opt.has_value() && ego_debug_opt->valid) {
    ego_line1 << fmtFloat(ego_debug_opt->ego_speed_mps, "m/s", 1)
              << "  pitch "
              << fmtFloat(ego_debug_opt->pitch_rad * 57.2957795f, "deg", 1);
  } else {
    ego_line1 << "waiting for ego state";
  }
  std::ostringstream ego_line2;
  ego_line2 << "fresh "
            << ageLabel(now_ns, ego_debug_opt.has_value()
                                    ? ego_debug_opt->timestamp_ns
                                    : 0ULL);
  drawCard("EGO / IMU", ego_line1.str(), ego_line2.str(),
           cv::Rect(420, 76, 182, 68), ego_live,
           cv::Scalar(110, 230, 170));

  const cv::Rect fusion_rect(54, 148, kPanelWidth - 108, 74);
  const cv::Rect gates_rect(36, 236, kPanelWidth - 72, 140);
  const cv::Rect risk_rect(72, 388, kPanelWidth - 144, 116);
  const cv::Rect state_rect(48, 516, kPanelWidth - 96, 74);
  const cv::Rect output_rect(138, 598, kPanelWidth - 276, 48);
  drawStageBox("Fusion / Track Merge", fusion_rect, cv::Scalar(86, 86, 94));
  drawStageBox("FCW Gates / Eligibility", gates_rect, cv::Scalar(86, 86, 94));
  drawStageBox("Risk Mix", risk_rect, cv::Scalar(86, 86, 94));
  drawStageBox("TTC / Escalation / Dwell", state_rect,
               cv::Scalar(86, 86, 94));
  drawStageBox("FCW Output", output_rect, cv::Scalar(86, 86, 94));

  const cv::Point camera_point(109, 144);
  const cv::Point radar_point(310, 144);
  const cv::Point ego_point(511, 144);
  const cv::Point fusion_point = flowStagePoint(adas::bev::FlowStage::Fusion);
  drawPulsedPolyline(panel, {camera_point, fusion_point},
                     cv::Scalar(255, 210, 120), 2, now_ns, snapshot_is_held,
                     camera_live ? 0.40 : 0.18);
  drawPulsedPolyline(panel, {radar_point, fusion_point},
                     cv::Scalar(90, 190, 255), 2, now_ns, snapshot_is_held,
                     radar_live ? 0.40 : 0.18);
  drawPulsedPolyline(panel, {ego_point, fusion_point},
                     cv::Scalar(120, 235, 180), 2, now_ns, snapshot_is_held,
                     ego_live ? 0.40 : 0.18);
  cv::circle(panel, fusion_point, 9, cv::Scalar(190, 190, 200), cv::FILLED,
             cv::LINE_AA);

  if (!debug_opt.has_value()) {
    if (snapshot_is_held && hold_remaining_s > 0.0f) {
      std::ostringstream ss;
      ss << "Holding last meaningful FCW path  " << std::fixed
         << std::setprecision(1) << hold_remaining_s << "s";
      drawPanelText(ss.str(), 20, 48, 0.31, cv::Scalar(220, 205, 170), 1);
    } else {
      drawPanelText("No FCW debug snapshot available yet", 20, 48, 0.31,
                    cv::Scalar(180, 180, 185), 1);
    }
    return;
  }

  const auto &debug = *debug_opt;
  const adas::FCWDebugCandidate *best =
      debug.has_best_candidate ? &debug.best_candidate : nullptr;
  const adas::FCWDebugCandidate *runner =
      debug.has_runner_up_candidate ? &debug.runner_up_candidate : nullptr;
  const cv::Scalar winner_color =
      best ? fcwLevelColor(best->active_level) : cv::Scalar(170, 170, 170);

  if (best) {
    std::ostringstream summary_line;
    summary_line << "Obj " << best->object_id << "  "
                 << adas::fcwRiskLevelName(best->active_level)
                 << "  risk " << std::fixed << std::setprecision(2)
                 << best->risk_score << "  TTC " << fmtFloat(best->ttc_s, "s", 1);
    drawPanelText(summary_line.str(), 20, 48, 0.34, winner_color, 1);
  } else if (snapshot_is_held && hold_remaining_s > 0.0f) {
    std::ostringstream ss;
    ss << "Holding last meaningful FCW path  " << std::fixed
       << std::setprecision(1) << hold_remaining_s << "s";
    drawPanelText(ss.str(), 20, 48, 0.31, cv::Scalar(220, 205, 170), 1);
  } else {
    drawPanelText("No eligible FCW candidate this tick", 20, 48, 0.34,
                  cv::Scalar(205, 205, 210), 1);
  }

  std::ostringstream fusion_summary;
  if (best) {
    fusion_summary << "Winner ID " << best->object_id
                   << "  fused cam+radar  Q " << std::fixed
                   << std::setprecision(2) << best->fusion_quality;
  } else {
    fusion_summary << "No object survived to the FCW path";
  }
  drawPanelText(fusion_summary.str(), fusion_rect.x + 12, fusion_rect.y + 38,
                0.34, cv::Scalar(235, 235, 240), 1);

  std::ostringstream fusion_detail;
  fusion_detail << "inputs  cam " << camera_count << "  radar " << radar_count
                << "  fused " << fused_count;
  if (runner) {
    fusion_detail << "  rival " << runner->object_id;
  }
  drawPanelText(fusion_detail.str(), fusion_rect.x + 12, fusion_rect.y + 58,
                0.30, cv::Scalar(180, 190, 205), 1);

  const std::array<std::pair<adas::bev::FlowStage, bool>, 8> gate_states = {{
      {adas::bev::FlowStage::CameraFresh, best ? best->gate_camera_ok : false},
      {adas::bev::FlowStage::HasRadar, best ? best->gate_has_radar : false},
      {adas::bev::FlowStage::SpeedFresh, best ? best->gate_speed_fresh : false},
      {adas::bev::FlowStage::ClassRelevant,
       best ? best->gate_class_relevant : false},
      {adas::bev::FlowStage::RangeOk, best ? best->gate_range_ok : false},
      {adas::bev::FlowStage::QualityOk, best ? best->gate_quality_ok : false},
      {adas::bev::FlowStage::ClosingOk, best ? best->gate_closing_ok : false},
      {adas::bev::FlowStage::InPath, best ? best->gate_in_path : false},
  }};
  for (const auto &gate : gate_states) {
    drawStageChip(gate.first, adas::bev::flowStageName(gate.first), gate.second,
                  winner_color);
  }

  if (best) {
    const std::vector<cv::Point> winner_lane = {
        fusion_point,
        flowStagePoint(adas::bev::FlowStage::CameraFresh),
        flowStagePoint(adas::bev::FlowStage::HasRadar),
        flowStagePoint(adas::bev::FlowStage::SpeedFresh),
        flowStagePoint(adas::bev::FlowStage::ClassRelevant),
        flowStagePoint(adas::bev::FlowStage::RangeOk),
        flowStagePoint(adas::bev::FlowStage::QualityOk),
        flowStagePoint(adas::bev::FlowStage::ClosingOk),
        flowStagePoint(adas::bev::FlowStage::InPath),
        flowStagePoint(adas::bev::FlowStage::RiskMix),
        flowStagePoint(adas::bev::FlowStage::StateMachine),
        flowStagePoint(adas::bev::FlowStage::Output)};
    drawPulsedPolyline(panel, winner_lane, winner_color, 3, now_ns,
                       snapshot_is_held,
                       snapshot_is_held ? 0.34 : 0.52);
  }

  if (runner && best) {
    const adas::bev::FlowLaneModel runner_lane =
        adas::bev::makeRunnerUpFlowLane(*best, *runner);
    const cv::Point stage_point = flowStagePoint(runner_lane.terminal_stage);
    const std::vector<cv::Point> rival_path = {
        fusion_point,
        cv::Point(540, fusion_point.y + 24),
        cv::Point(540, stage_point.y),
        stage_point,
        cv::Point(580, stage_point.y)};
    drawPulsedPolyline(panel, rival_path, cv::Scalar(0, 190, 255), 2, now_ns,
                       snapshot_is_held, 0.28);
    cv::rectangle(panel, cv::Rect(430, stage_point.y - 14, 160, 28),
                  cv::Scalar(28, 34, 40), cv::FILLED);
    cv::rectangle(panel, cv::Rect(430, stage_point.y - 14, 160, 28),
                  cv::Scalar(0, 130, 190), 1);
    std::ostringstream runner_label;
    runner_label << "ID " << runner->object_id << "  "
                 << runner_lane.terminal_label;
    drawPanelText(runner_label.str(), 438, stage_point.y + 5, 0.29,
                  cv::Scalar(205, 225, 240), 1);
  }

  for (size_t i = 0; i < std::min<size_t>(2, debug.rejected_candidates.size());
       ++i) {
    const auto lane = adas::bev::makeRejectedFlowLane(debug.rejected_candidates[i]);
    const cv::Point stage_point = flowStagePoint(lane.terminal_stage);
    const int gutter_x = (i % 2 == 0) ? 46 : 574;
    const std::vector<cv::Point> reject_path = {
        fusion_point,
        cv::Point(stage_point.x, fusion_point.y + 18),
        stage_point,
        cv::Point(gutter_x, stage_point.y)};
    drawPulsedPolyline(panel, reject_path, cv::Scalar(80, 100, 255), 2, now_ns,
                       snapshot_is_held, 0.22);

    const int label_width = 150;
    const int label_x =
        (gutter_x < stage_point.x) ? 18 : (kPanelWidth - 18 - label_width);
    cv::rectangle(panel,
                  cv::Rect(label_x, stage_point.y - 15, label_width, 30),
                  cv::Scalar(38, 28, 34), cv::FILLED);
    cv::rectangle(panel,
                  cv::Rect(label_x, stage_point.y - 15, label_width, 30),
                  cv::Scalar(90, 110, 255), 1);
    std::ostringstream reject_line;
    reject_line << "ID " << debug.rejected_candidates[i].object_id;
    drawPanelText(reject_line.str(), label_x + 8, stage_point.y - 1, 0.29,
                  cv::Scalar(240, 230, 230), 1);
    drawPanelText(lane.terminal_label, label_x + 8, stage_point.y + 12, 0.28,
                  cv::Scalar(220, 185, 185), 1);
  }

  if (best) {
    drawBar(risk_rect.x + 16, risk_rect.y + 27, risk_rect.width - 32,
            "Closing", best->closing_score, 0.40f,
            cv::Scalar(70, 140, 255));
    drawBar(risk_rect.x + 16, risk_rect.y + 43, risk_rect.width - 32,
            "Range", best->range_score, 0.24f, cv::Scalar(70, 200, 255));
    drawBar(risk_rect.x + 16, risk_rect.y + 59, risk_rect.width - 32,
            "Quality", best->quality_score, 0.21f,
            cv::Scalar(120, 220, 120));
    drawBar(risk_rect.x + 16, risk_rect.y + 75, risk_rect.width - 32, "TTC",
            best->ttc_score, 0.10f, cv::Scalar(0, 215, 235));
    drawBar(risk_rect.x + 16, risk_rect.y + 91, risk_rect.width - 32,
            "Physics", best->physics_score, 0.05f,
            cv::Scalar(120, 120, 255));
    drawBar(risk_rect.x + 16, risk_rect.y + 107, risk_rect.width - 32,
            "Final", best->risk_score, 1.00f, winner_color);
  }

  if (best) {
    int badge_x = state_rect.x + 14;
    int badge_y = state_rect.y + 24;
    badge_x += drawBadge(badge_x, badge_y,
                         std::string("Base ") +
                             adas::fcwRiskLevelName(best->base_level),
                         true, fcwLevelColor(best->base_level));
    badge_x += drawBadge(badge_x, badge_y, "TTC caution",
                         best->ttc_caution_floor, cv::Scalar(0, 215, 235));
    badge_x += drawBadge(badge_x, badge_y, "Warn floor",
                         best->ttc_warn_floor, cv::Scalar(0, 165, 255));
    badge_x = state_rect.x + 14;
    badge_y += 22;
    badge_x += drawBadge(badge_x, badge_y, "Last ditch",
                         best->ttc_last_ditch, cv::Scalar(0, 125, 255));
    badge_x += drawBadge(badge_x, badge_y, "Imm warn",
                         best->ttc_immediate_warn, cv::Scalar(0, 165, 255));
    badge_x += drawBadge(badge_x, badge_y, "Imm crit",
                         best->ttc_immediate_critical, cv::Scalar(0, 0, 255));
    badge_x += drawBadge(badge_x, badge_y, "Cam grace",
                         best->camera_drop_grace_used,
                         cv::Scalar(180, 110, 255));

    std::ostringstream geo_line;
    geo_line << "Path X " << std::fixed << std::setprecision(2)
             << best->x_lateral_m << "m / half " << best->lane_half_width_m
             << "m    Stop " << best->stopping_distance_m << "m / range "
             << best->range_m << "m";
    drawPanelText(geo_line.str(), state_rect.x + 14, state_rect.y + 64, 0.29,
                  cv::Scalar(195, 210, 220), 1);
  } else {
    drawPanelText("No candidate reached the FCW state machine this tick",
                  state_rect.x + 14, state_rect.y + 46, 0.34,
                  cv::Scalar(180, 180, 185), 1);
  }

  if (best) {
    std::ostringstream output_line;
    output_line << adas::fcwRiskLevelName(best->active_level) << "   Obj "
                << best->object_id << "   TTC "
                << fmtFloat(best->ttc_s, "s", 1) << "   BLE ";
    if (ble_send_active) {
      output_line << "YES";
    } else if (snapshot_is_held) {
      output_line << "HELD";
    } else {
      output_line << "NO";
    }
    drawPanelText(output_line.str(), output_rect.x + 16, output_rect.y + 32,
                  0.38, winner_color, 1);
  } else {
    drawPanelText("SAFE   No FCW output", output_rect.x + 16,
                  output_rect.y + 32, 0.38, cv::Scalar(175, 175, 180), 1);
  }
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

  if (frame.has_camera_batch) {
    latest_camera_source_.count = static_cast<int>(frame.camera_batch.dets.size());
    latest_camera_source_.timestamp_ns =
        frame.camera_batch.h.t_ingest_ns != 0 ? frame.camera_batch.h.t_ingest_ns
                                              : now_ns;
    latest_camera_source_.healthy = frame.camera_batch.h.healthy;
  }

  if (frame.has_radar_targets) {
    latest_radar_source_.count =
        static_cast<int>(frame.radar_targets.targets.size());
    latest_radar_source_.timestamp_ns =
        frame.radar_targets.h.t_ingest_ns != 0 ? frame.radar_targets.h.t_ingest_ns
                                               : now_ns;
    latest_radar_source_.healthy = frame.radar_targets.h.healthy;
  }

  if (frame.ego_debug_context.has_value()) {
    latest_ego_debug_snapshot_ = frame.ego_debug_context;
    if (latest_ego_debug_snapshot_->timestamp_ns == 0) {
      latest_ego_debug_snapshot_->timestamp_ns = now_ns;
    }
  }

  for (const auto &det : frame.camera_batch.dets) {
    if (det.object_id == UINT64_MAX) {
      continue;
    }

    const float angle_rad =
        adas::bev::angleFromPixel(det.centroid.x, c_x_, f_x_);

    Track &track = tracks_[det.object_id];
    track.object_id = det.object_id;
    track.corridor_angle_rad = angle_rad;
    track.last_cam_update_ns = now_ns;
    track.has_cam_est_range = false;
    track.cam_est_range_m = 0.0f;
    track.has_crosshair = false;

    const int conf_pct = static_cast<int>(std::round(det.score * 100.0f));
    std::stringstream ss;
    ss << "[" << det.object_id << "] C:" << std::clamp(conf_pct, 0, 100) << "%";
    track.label = ss.str();
  }

  for (const auto &obj : frame.fused_objects) {
    if (obj.object_id == UINT64_MAX) {
      continue;
    }

    Track &track = tracks_[obj.object_id];
    track.object_id = obj.object_id;

    const float angle_rad = obj.theta_rad;
    track.corridor_angle_rad = angle_rad;

    if (obj.camera_age_ms < (std::numeric_limits<uint32_t>::max() / 2)) {
      const uint64_t cam_age_ns =
          static_cast<uint64_t>(obj.camera_age_ms) * 1000000ULL;
      track.last_cam_update_ns =
          (now_ns > cam_age_ns) ? (now_ns - cam_age_ns) : now_ns;
    }
    track.camera_age_ms = obj.camera_age_ms;
    track.radar_age_ms = obj.radar_age_ms;
    track.is_predicted_camera = obj.is_predicted_camera;
    track.is_aggressive_mode = obj.is_aggressive_mode;
    track.ttc_s = obj.ttc_s;
    track.fusion_quality = obj.fusion_quality;
    track.speed_fresh = obj.speed_fresh;
    track.speed_age_ms = obj.speed_age_ms;
    track.radial_vel_mps = obj.radial_vel_mps;
    track.has_cam_est_range = obj.z_cam_m > 0.1f;
    track.cam_est_range_m = obj.z_cam_m;
    track.dz_cam_radar_m = -1.0f;

    if (obj.has_radar && obj.range_m > 0.1f) {
      track.has_radar = true;
      const uint64_t radar_age_ns =
          static_cast<uint64_t>(obj.radar_age_ms) * 1000000ULL;
      track.last_range_update_ns =
          (now_ns > radar_age_ns) ? (now_ns - radar_age_ns) : now_ns;
      track.z_m = obj.range_m;
      if (track.has_cam_est_range) {
        track.dz_cam_radar_m = std::abs(obj.range_m - track.cam_est_range_m);
      }
      // Use pipeline-computed fused lateral position directly.
      track.x_offset_m = obj.x_lateral_m;
      track.has_crosshair = true;
    } else {
      track.has_radar = false;
      track.has_crosshair = false;
    }

    std::stringstream ss;
    ss << "[" << obj.object_id << "] ";
    if (obj.is_predicted_camera) {
      ss << "P ";
    }
    if (obj.is_aggressive_mode) {
      ss << "AG ";
    }
    if (obj.has_radar && obj.range_m > 0.1f) {
      ss << "Z:" << std::fixed << std::setprecision(1) << obj.range_m << "m";
      if (track.speed_fresh) {
        ss << " V:" << static_cast<int>(std::round(obj.radial_vel_mps))
           << "m/s";
      } else {
        ss << " V:stale";
      }
    } else if (track.has_cam_est_range) {
      ss << "Zcam:" << std::fixed << std::setprecision(1)
         << track.cam_est_range_m << "m";
    } else {
      ss << "Zcam:unknown";
    }
    track.label = ss.str();
  }

  if (frame.fcw_alert_context.has_value() &&
      frame.fcw_alert_context->object_id != UINT64_MAX) {
    auto it = tracks_.find(frame.fcw_alert_context->object_id);
    if (it != tracks_.end()) {
      const uint64_t hold_ns = static_cast<uint64_t>(ttc_hold_ms_) * 1000000ULL;
      it->second.ttc_hold_until_ns =
          std::max(it->second.ttc_hold_until_ns, now_ns + hold_ns);
      // Copy trigger speed directly from FCW pipeline output.
      it->second.has_fcw_trigger_speed = true;
      it->second.fcw_trigger_speed_mps = frame.fcw_alert_context->velocity_mps;
    }
  } else if (frame.fcw_focus_object_id.has_value()) {
    auto it = tracks_.find(*frame.fcw_focus_object_id);
    if (it != tracks_.end()) {
      const uint64_t hold_ns = static_cast<uint64_t>(ttc_hold_ms_) * 1000000ULL;
      it->second.ttc_hold_until_ns =
          std::max(it->second.ttc_hold_until_ns, now_ns + hold_ns);
    }
  }

  if (frame.fcw_eval_context.has_value() &&
      frame.fcw_eval_context->has_candidate &&
      frame.fcw_eval_context->object_id != UINT64_MAX) {
    auto it = tracks_.find(frame.fcw_eval_context->object_id);
    if (it != tracks_.end()) {
      it->second.fcw_eval_level = frame.fcw_eval_context->level;
      it->second.fcw_eval_risk = frame.fcw_eval_context->risk_score;
      it->second.fcw_eval_used_camera_drop_grace =
          frame.fcw_eval_context->used_camera_drop_grace;
      it->second.fcw_eval_until_ns = now_ns + 600000000ULL; // 600 ms
    }
  }
}

void BEVDashboard::renderLoop() {
  cv::namedWindow("ADAS BEVDashboard", cv::WINDOW_AUTOSIZE);
  cv::Mat radar_heat_accumulator =
      cv::Mat::zeros(kCanvasHeight, kCanvasWidth, CV_32FC1);

  uint64_t last_processed_seq = 0;

  while (running_.load()) {
    cv::Mat world(kCanvasHeight, kCanvasWidth, CV_8UC3,
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
    const uint32_t dead_track_cleanup_ms =
        dead_track_cleanup_ms_.load(std::memory_order_relaxed);
    const bool incoming_has_display_content =
        snapshotHasDisplayContent(frame.fcw_debug_context);
    if (incoming_has_display_content) {
      const bool have_latched = latched_fcw_debug_snapshot_.has_value();
      const bool same_best_object =
          have_latched &&
          snapshotBestId(*latched_fcw_debug_snapshot_) ==
              snapshotBestId(*frame.fcw_debug_context);
      const bool escalated_level =
          have_latched && frame.fcw_debug_context->has_best_candidate &&
          (!latched_fcw_debug_snapshot_->has_best_candidate ||
           static_cast<int>(frame.fcw_debug_context->best_candidate.active_level) >
               static_cast<int>(
                   latched_fcw_debug_snapshot_->best_candidate.active_level));
      const bool refresh_allowed =
          !have_latched ||
          (now_ns - latched_fcw_debug_last_update_ns_) >=
              kPanelSnapshotMinRefreshNs;

      if (same_best_object || escalated_level || refresh_allowed) {
        latched_fcw_debug_snapshot_ = frame.fcw_debug_context;
        latched_fcw_debug_last_update_ns_ = now_ns;
      }
      latched_fcw_debug_until_ns_ = now_ns + kPanelSnapshotHoldNs;
    } else if (latched_fcw_debug_until_ns_ <= now_ns) {
      latched_fcw_debug_snapshot_.reset();
    }

    const bool use_latched_snapshot = latched_fcw_debug_snapshot_.has_value() &&
                                      latched_fcw_debug_until_ns_ > now_ns;
    const float held_remaining_s =
        use_latched_snapshot
            ? static_cast<float>(latched_fcw_debug_until_ns_ - now_ns) /
                  1.0e9f
            : 0.0f;

    bool left_bsd = false;
    bool right_bsd = false;
    if (bsd_receiver_) {
      left_bsd = bsd_receiver_->getLeftBSDState();
      right_bsd = bsd_receiver_->getRightBSDState();
    }

    drawBlindSpotZones(world, left_bsd, right_bsd);
    drawEgoVehicle(world);
    drawRadarCone(world);
    drawRadarHeatmap(world, radar_heat_accumulator, frame.radar_targets,
                     has_new_frame);

    // Raw radar scan-lines (20 degree cone)
    for (const auto &target : frame.radar_targets.targets) {
      drawRadarRangeLine(world, target.range_m, cv::Scalar(190, 110, 30), 1);
    }

    struct FcwRayOverlay {
      cv::Point target;
      bool has_trigger_speed = false;
      float trigger_speed_mps = 0.0f;
    };
    std::vector<FcwRayOverlay> fcw_ray_overlays;
    int aggressive_tracks_visible = 0;
    float best_ttc_s = std::numeric_limits<float>::infinity();
    uint64_t best_ttc_id = UINT64_MAX;

    for (auto it = tracks_.begin(); it != tracks_.end();) {
      Track &track = it->second;

      if (!adas::bev::shouldKeepTrack(
              now_ns, track.last_cam_update_ns, track.last_range_update_ns,
              track.ttc_hold_until_ns, dead_track_cleanup_ms)) {
        it = tracks_.erase(it);
        continue;
      }

      const float cam_remain = adas::bev::ttlRemaining01(
          now_ns, track.last_cam_update_ns, dead_track_cleanup_ms);
      const float range_remain = adas::bev::ttlRemaining01(
          now_ns, track.last_range_update_ns, dead_track_cleanup_ms);
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
      const bool speed_vector_active =
          track.speed_fresh && std::abs(track.radial_vel_mps) > 0.1f;
      if ((cam_alive || range_alive) && track.is_aggressive_mode) {
        ++aggressive_tracks_visible;
      }
      if ((cam_alive || range_alive) && std::isfinite(track.ttc_s) &&
          track.ttc_s > 0.0f && track.ttc_s < best_ttc_s) {
        best_ttc_s = track.ttc_s;
        best_ttc_id = track.object_id;
      }

      // Camera corridor layer (25 degree span)
      if (cam_alive &&
          adas::bev::inAngleSpan(track.corridor_angle_rad, kCameraHalfFovDeg)) {
        const cv::Scalar corridor_color =
            crosshair_active ? (speed_vector_active ? cv::Scalar(120, 220, 120)
                                                    : cv::Scalar(0, 220, 255))
                             : cv::Scalar(90, 90, 170); // ghost
        drawCorridor(world, track.corridor_angle_rad, corridor_color,
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
          // Bright fused-position dot for immediate object localization.
          cv::circle(world, cv::Point(anchor_x, anchor_y), 9, marker_color, 1,
                     cv::LINE_AA);
          cv::circle(world, cv::Point(anchor_x, anchor_y), 5, marker_color,
                     cv::FILLED, cv::LINE_AA);
          drawCrosshair(world, anchor_x, anchor_y, marker_color, 6, 2);

          if (hold_active) {
            cv::circle(world, cv::Point(anchor_x, anchor_y), 10,
                       cv::Scalar(0, 0, 180), 2);
          }
          if (track.is_aggressive_mode) {
            cv::circle(world, cv::Point(anchor_x, anchor_y), 14,
                       cv::Scalar(0, 165, 255), 2);
            cv::putText(world, "AGG", cv::Point(anchor_x + 8, anchor_y + 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(0, 165, 255),
                        1, cv::LINE_AA);
          }

          if (speed_vector_active) {
            const int arrow_len = 12;
            const int dir = (track.radial_vel_mps > 0.0f) ? 1 : -1;
            cv::arrowedLine(world, cv::Point(anchor_x, anchor_y),
                            cv::Point(anchor_x, anchor_y + dir * arrow_len),
                            cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.35);
          }
        }
      } else if (range_alive && !cam_alive && track.z_m > 0.1f) {
        // Range-only track visualization fallback.
        anchor_x = toCanvasX(track.x_offset_m);
        anchor_y = toCanvasY(track.z_m);
        cv::circle(world, cv::Point(anchor_x, anchor_y), 4,
                   cv::Scalar(200, 110, 30), cv::FILLED);
      }

      if (cam_alive && track.has_cam_est_range &&
          track.cam_est_range_m > 0.1f &&
          track.cam_est_range_m <= maxDisplayRangeM()) {
        // Camera-estimated pose from pipeline math:
        // X from centroid angle, Y from z_cam_m.
        const float x_cam = adas::bev::lateralFromRangeAndAngle(
            track.cam_est_range_m, track.corridor_angle_rad);
        const int cam_dot_x = toCanvasX(x_cam);
        const int cam_dot_y = toCanvasY(track.cam_est_range_m);
        if (cam_dot_x >= 0 && cam_dot_x < kCanvasWidth && cam_dot_y >= 0 &&
            cam_dot_y < kCanvasHeight) {
          const bool outside_radar_cone = !adas::bev::inAngleSpan(
              track.corridor_angle_rad, kRadarHalfFovDeg);
          if (outside_radar_cone) {
            drawCorridor(world, track.corridor_angle_rad,
                         cv::Scalar(95, 95, 95), 1);
          }
          cv::circle(world, cv::Point(cam_dot_x, cam_dot_y), 4,
                     outside_radar_cone ? cv::Scalar(135, 135, 135)
                                        : cv::Scalar(255, 210, 120),
                     cv::FILLED);
        }
      }

      // TTL bars: stale cleanup + TTC hold timer.
      drawProgressBar(world, anchor_x - 18, anchor_y - 18, 36, 4,
                      std::max(cam_remain, range_remain),
                      cv::Scalar(0, 180, 230));
      if (hold_active) {
        drawProgressBar(world, anchor_x - 18, anchor_y - 24, 36, 4,
                        std::clamp(hold_remain, 0.0f, 1.0f),
                        cv::Scalar(0, 0, 255));
      }

      // Text label and speed freshness note.
      const cv::Scalar text_color = track.speed_fresh
                                        ? cv::Scalar(230, 230, 230)
                                        : cv::Scalar(160, 160, 160);
      cv::putText(world, track.label, cv::Point(anchor_x - 40, anchor_y - 40),
                  cv::FONT_HERSHEY_SIMPLEX, 0.32, text_color, 1);

      if (track.fcw_eval_until_ns > now_ns) {
        cv::Scalar eval_color(150, 150, 150); // Safe
        if (track.fcw_eval_level == 1) {
          eval_color = cv::Scalar(0, 220, 220); // Caution
        } else if (track.fcw_eval_level == 2) {
          eval_color = cv::Scalar(0, 165, 255); // Warn
        } else if (track.fcw_eval_level >= 3) {
          eval_color = cv::Scalar(0, 0, 255); // Critical
        }

        std::stringstream eval_ss;
        eval_ss << "FCW L" << static_cast<int>(track.fcw_eval_level)
                << " r=" << std::fixed << std::setprecision(2)
                << track.fcw_eval_risk;
        if (track.fcw_eval_used_camera_drop_grace) {
          eval_ss << " RG";
        }
        cv::putText(world, eval_ss.str(),
                    cv::Point(anchor_x - 40, anchor_y - 29),
                    cv::FONT_HERSHEY_SIMPLEX, 0.30, eval_color, 1, cv::LINE_AA);
      }

      std::stringstream dbg1;
      dbg1 << "TTC:";
      if (std::isfinite(track.ttc_s) && track.ttc_s > 0.0f) {
        dbg1 << std::fixed << std::setprecision(2) << track.ttc_s << "s";
      } else {
        dbg1 << "--";
      }
      dbg1 << " Q:" << std::fixed << std::setprecision(2)
           << track.fusion_quality << " dZ:";
      if (track.dz_cam_radar_m >= 0.0f) {
        dbg1 << std::fixed << std::setprecision(2) << track.dz_cam_radar_m;
      } else {
        dbg1 << "--";
      }
      cv::putText(world, dbg1.str(), cv::Point(anchor_x - 40, anchor_y - 18),
                  cv::FONT_HERSHEY_SIMPLEX, 0.28, cv::Scalar(180, 210, 210), 1,
                  cv::LINE_AA);

      const auto ageToText = [](uint32_t age_ms) {
        if (age_ms >= (std::numeric_limits<uint32_t>::max() / 2)) {
          return std::string("--");
        }
        return std::to_string(age_ms);
      };
      std::stringstream dbg2;
      dbg2 << "age c:" << ageToText(track.camera_age_ms)
           << " r:" << ageToText(track.radar_age_ms)
           << " sp:" << ageToText(track.speed_age_ms)
           << " P:" << (track.is_predicted_camera ? "1" : "0")
           << " AG:" << (track.is_aggressive_mode ? "1" : "0");
      cv::putText(world, dbg2.str(), cv::Point(anchor_x - 40, anchor_y - 7),
                  cv::FONT_HERSHEY_SIMPLEX, 0.27, cv::Scalar(170, 170, 170), 1,
                  cv::LINE_AA);

      if (hold_active && track.z_m > 0.1f && anchor_x >= 0 &&
          anchor_x < kCanvasWidth && anchor_y >= 0 &&
          anchor_y < kCanvasHeight) {
        FcwRayOverlay overlay;
        overlay.target = cv::Point(anchor_x, anchor_y);
        overlay.has_trigger_speed = track.has_fcw_trigger_speed;
        overlay.trigger_speed_mps = track.fcw_trigger_speed_mps;
        fcw_ray_overlays.push_back(overlay);
      }

      ++it;
    }

    cv::rectangle(world, cv::Rect(6, 6, 235, 52), cv::Scalar(35, 35, 35),
                  cv::FILLED);
    cv::rectangle(world, cv::Rect(6, 6, 235, 52), cv::Scalar(90, 90, 90), 1);
    cv::putText(world, "World Model", cv::Point(10, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(230, 230, 230), 1,
                cv::LINE_AA);

    std::stringstream summary_1;
    summary_1 << "AG tracks: " << aggressive_tracks_visible << "  Best TTC: ";
    if (std::isfinite(best_ttc_s)) {
      summary_1 << std::fixed << std::setprecision(2) << best_ttc_s << "s"
                << " ID:" << best_ttc_id;
    } else {
      summary_1 << "--";
    }
    cv::putText(world, summary_1.str(), cv::Point(10, 32),
                cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(0, 200, 255), 1,
                cv::LINE_AA);

    if (frame.fcw_eval_context.has_value() &&
        frame.fcw_eval_context->has_candidate) {
      std::stringstream summary_2;
      summary_2 << "FCW cand ID:" << frame.fcw_eval_context->object_id
                << " L:" << static_cast<int>(frame.fcw_eval_context->level)
                << " r:" << std::fixed << std::setprecision(2)
                << frame.fcw_eval_context->risk_score;
      cv::putText(world, summary_2.str(), cv::Point(10, 46),
                  cv::FONT_HERSHEY_SIMPLEX, 0.28, cv::Scalar(180, 220, 255), 1,
                  cv::LINE_AA);
    }

    // FCW directional rays are drawn last so they remain visible over overlays.
    for (const auto &overlay : fcw_ray_overlays) {
      drawFcwDirectionRay(world, overlay.target);
      if (overlay.has_trigger_speed) {
        std::stringstream ss;
        ss << "FCW v=" << std::fixed << std::setprecision(1)
           << overlay.trigger_speed_mps << "m/s";
        cv::putText(world, ss.str(),
                    cv::Point(overlay.target.x + 8, overlay.target.y - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(235, 235, 255),
                    1, cv::LINE_AA);
      }
    }

    cv::Mat panel;
    renderFcwThoughtFlowPanel(
        panel,
        use_latched_snapshot ? latched_fcw_debug_snapshot_
                             : frame.fcw_debug_context,
        use_latched_snapshot && !incoming_has_display_content, held_remaining_s,
        latest_camera_source_.count, latest_camera_source_.timestamp_ns,
        latest_camera_source_.healthy, latest_radar_source_.count,
        latest_radar_source_.timestamp_ns, latest_radar_source_.healthy,
        latest_ego_debug_snapshot_, static_cast<int>(frame.fused_objects.size()),
        frame.fcw_alert_context.has_value(), now_ns);

    cv::Mat canvas;
    cv::hconcat(std::vector<cv::Mat>{world, panel}, canvas);

    cv::Mat display_canvas = canvas;
    const double scale_x =
        static_cast<double>(kDisplayMaxWidth) / static_cast<double>(canvas.cols);
    const double scale_y = static_cast<double>(kDisplayMaxHeight) /
                           static_cast<double>(canvas.rows);
    const double display_scale = std::min({1.0, scale_x, scale_y});
    if (display_scale < 0.999) {
      cv::resize(canvas, display_canvas, cv::Size(), display_scale,
                 display_scale, cv::INTER_AREA);
    }

    cv::imshow("ADAS BEVDashboard", display_canvas);
    cv::waitKey(33);
  }

  try {
    cv::destroyWindow("ADAS BEVDashboard");
  } catch (...) {
  }
}

} // namespace adas
