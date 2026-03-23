// File: src/stage_e/BEVDashboard.cpp
#include "adas/stage_e/BEVDashboard.hpp"

#include "adas/common/Clock.hpp"
#include "adas/stage_e/BEVDashboardMath.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

namespace {

constexpr int kCanvasWidth = 520;
constexpr int kCanvasHeight = 660;
constexpr int kPanelWidth = 620;
constexpr int kPanelHeight = kCanvasHeight;
constexpr int kDisplayMaxWidth = 960;
constexpr int kDisplayMaxHeight = 520;
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

void renderFcwReasoningPanel(
    cv::Mat &panel, const std::optional<adas::FCWDebugSnapshot> &debug_opt) {
  panel = cv::Mat(kPanelHeight, kPanelWidth, CV_8UC3, cv::Scalar(24, 24, 26));

  const auto drawPanelText = [&](const std::string &text, int x, int y,
                                 double scale, const cv::Scalar &color,
                                 int thickness = 1) {
    cv::putText(panel, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv::LINE_AA);
  };

  const auto drawSectionHeader = [&](const std::string &title, int y) {
    cv::rectangle(panel, cv::Rect(12, y - 16, kPanelWidth - 24, 24),
                  cv::Scalar(38, 38, 42), cv::FILLED);
    drawPanelText(title, 18, y, 0.42, cv::Scalar(240, 240, 240), 1);
  };

  const auto drawBadge =
      [&](int x, int y, const std::string &label, bool active,
          const cv::Scalar &color) -> int {
    int baseline = 0;
    const cv::Size text_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.33, 1, &baseline);
    const int pad_x = 8;
    const int width = text_size.width + pad_x * 2;
    const int height = 18;
    const cv::Scalar fill =
        active ? color : cv::Scalar(55, 55, 58);
    const cv::Scalar edge =
        active ? color : cv::Scalar(95, 95, 100);
    cv::rectangle(panel, cv::Rect(x, y, width, height), fill, cv::FILLED);
    cv::rectangle(panel, cv::Rect(x, y, width, height), edge, 1);
    drawPanelText(label, x + pad_x, y + 13, 0.33,
                  active ? cv::Scalar(245, 245, 245)
                         : cv::Scalar(150, 150, 155),
                  1);
    return width + 6;
  };

  const auto drawBar =
      [&](int x, int y, int width, const std::string &label, float value,
          float weight, const cv::Scalar &color) {
    const float clamped = std::clamp(value, 0.0f, 1.0f);
    cv::rectangle(panel, cv::Rect(x, y, width, 12), cv::Scalar(42, 42, 46),
                  cv::FILLED);
    cv::rectangle(panel, cv::Rect(x, y, width, 12), cv::Scalar(90, 90, 96), 1);
    cv::rectangle(panel, cv::Rect(x, y, static_cast<int>(width * clamped), 12),
                  color, cv::FILLED);

    std::ostringstream rhs;
    rhs << std::fixed << std::setprecision(2) << value << " x" << weight
        << " = " << (value * weight);
    drawPanelText(label, x, y - 4, 0.31, cv::Scalar(205, 205, 210), 1);
    drawPanelText(rhs.str(), x + width - 175, y - 4, 0.31,
                  cv::Scalar(175, 175, 180), 1);
  };

  const auto drawCandidateCard =
      [&](const adas::FCWDebugCandidate &cand, const std::string &title, int x,
          int y, int width, int height, bool is_primary) {
    const cv::Scalar accent = fcwLevelColor(cand.active_level);
    cv::rectangle(panel, cv::Rect(x, y, width, height), cv::Scalar(31, 31, 35),
                  cv::FILLED);
    cv::rectangle(panel, cv::Rect(x, y, width, height),
                  is_primary ? accent : cv::Scalar(90, 90, 95),
                  is_primary ? 2 : 1);
    cv::rectangle(panel, cv::Rect(x, y, width, 20), accent, cv::FILLED);
    drawPanelText(title, x + 8, y + 14, 0.36, cv::Scalar(248, 248, 248), 1);

    std::ostringstream id_line;
    id_line << "ID " << cand.object_id << "  "
            << adas::fcwRiskLevelName(cand.active_level);
    drawPanelText(id_line.str(), x + 8, y + 38, 0.40,
                  cv::Scalar(235, 235, 240), 1);

    std::ostringstream line1;
    line1 << "Risk " << std::fixed << std::setprecision(2) << cand.risk_score
          << "  TTC " << fmtFloat(cand.ttc_s, "s")
          << "  Z " << fmtFloat(cand.range_m, "m");
    drawPanelText(line1.str(), x + 8, y + 58, 0.34,
                  cv::Scalar(215, 215, 220), 1);

    std::ostringstream line2;
    line2 << "V " << fmtFloat(cand.velocity_mps, "m/s")
          << "  X " << fmtFloat(cand.x_lateral_m, "m")
          << "  Q " << fmtFloat(cand.fusion_quality, "", 2);
    drawPanelText(line2.str(), x + 8, y + (height >= 100 ? 77 : 74), 0.34,
                  cv::Scalar(195, 195, 200), 1);

    if (height >= 100) {
      std::ostringstream line3;
      line3 << "Base/Des/Act "
            << adas::fcwRiskLevelName(cand.base_level) << " / "
            << adas::fcwRiskLevelName(cand.desired_level) << " / "
            << adas::fcwRiskLevelName(cand.active_level);
      drawPanelText(line3.str(), x + 8, y + 96, 0.32,
                    cv::Scalar(170, 200, 220), 1);
    }

    if (!cand.comparison_reason.empty()) {
      drawPanelText("Lost: " + cand.comparison_reason, x + 8, y + height - 8,
                    0.31, cv::Scalar(255, 205, 150), 1);
    }
  };

  const auto drawPathGauge = [&](int x, int y, int width,
                                 const adas::FCWDebugCandidate &cand) {
    const float max_extent =
        std::max({std::abs(cand.x_lateral_m), cand.lane_half_width_m, 0.5f}) *
        1.35f;
    const int h = 16;
    cv::rectangle(panel, cv::Rect(x, y, width, h), cv::Scalar(40, 40, 44),
                  cv::FILLED);
    cv::rectangle(panel, cv::Rect(x, y, width, h), cv::Scalar(90, 90, 95), 1);
    const int center_x = x + width / 2;
    const int safe_half_px = static_cast<int>(
        std::clamp(cand.lane_half_width_m / max_extent, 0.0f, 1.0f) *
        (width / 2));
    cv::rectangle(panel,
                  cv::Rect(center_x - safe_half_px, y, safe_half_px * 2, h),
                  cv::Scalar(45, 110, 65), cv::FILLED);
    cv::line(panel, cv::Point(center_x, y), cv::Point(center_x, y + h),
             cv::Scalar(210, 210, 210), 1);
    const int marker_x = center_x + static_cast<int>(
                                        std::clamp(cand.x_lateral_m / max_extent,
                                                   -1.0f, 1.0f) *
                                        (width / 2));
    cv::line(panel, cv::Point(marker_x, y - 2), cv::Point(marker_x, y + h + 2),
             cand.gate_in_path ? cv::Scalar(255, 255, 255)
                               : cv::Scalar(0, 0, 255),
             2);
    std::ostringstream ss;
    ss << "Path gate  X=" << std::fixed << std::setprecision(2)
       << cand.x_lateral_m << "m  half=" << cand.lane_half_width_m << "m";
    drawPanelText(ss.str(), x, y - 4, 0.31, cv::Scalar(210, 210, 214), 1);
  };

  const auto drawStoppingGauge = [&](int x, int y, int width,
                                     const adas::FCWDebugCandidate &cand) {
    const float scale_max =
        std::max({cand.range_m, cand.stopping_distance_m, 1.0f}) * 1.1f;
    const int h = 16;
    cv::rectangle(panel, cv::Rect(x, y, width, h), cv::Scalar(40, 40, 44),
                  cv::FILLED);
    cv::rectangle(panel, cv::Rect(x, y, width, h), cv::Scalar(90, 90, 95), 1);
    const int stop_x = x + static_cast<int>(
                               std::clamp(cand.stopping_distance_m / scale_max,
                                          0.0f, 1.0f) *
                               width);
    const int range_x = x + static_cast<int>(
                                std::clamp(cand.range_m / scale_max, 0.0f, 1.0f) *
                                width);
    cv::line(panel, cv::Point(stop_x, y - 2), cv::Point(stop_x, y + h + 2),
             cv::Scalar(0, 215, 235), 2);
    cv::line(panel, cv::Point(range_x, y - 2), cv::Point(range_x, y + h + 2),
             cand.physics_contributed ? cv::Scalar(0, 0, 255)
                                      : cv::Scalar(0, 255, 180),
             2);
    if (cand.physics_contributed && range_x < stop_x) {
      cv::rectangle(panel, cv::Rect(range_x, y, stop_x - range_x, h),
                    cv::Scalar(45, 45, 150), cv::FILLED);
    }
    std::ostringstream ss;
    ss << "Stopping distance  range=" << std::fixed << std::setprecision(2)
       << cand.range_m << "m  stop=" << cand.stopping_distance_m << "m";
    drawPanelText(ss.str(), x, y - 4, 0.31, cv::Scalar(210, 210, 214), 1);
  };

  drawSectionHeader("FCW Heuristic", 22);
  drawPanelText("Live reasoning from the production FCW math", 18, 40, 0.34,
                cv::Scalar(180, 180, 185), 1);

  if (!debug_opt.has_value()) {
    drawPanelText("No FCW debug snapshot available", 18, 84, 0.42,
                  cv::Scalar(220, 220, 225), 1);
    return;
  }

  const auto &debug = *debug_opt;

  drawSectionHeader("Who Is Winning?", 68);
  if (debug.has_best_candidate) {
    drawCandidateCard(debug.best_candidate, "Best Candidate", 18, 78,
                      kPanelWidth - 36, 104, true);
  } else {
    cv::rectangle(panel, cv::Rect(18, 78, kPanelWidth - 36, 72),
                  cv::Scalar(31, 31, 35), cv::FILLED);
    cv::rectangle(panel, cv::Rect(18, 78, kPanelWidth - 36, 72),
                  cv::Scalar(90, 90, 95), 1);
    drawPanelText("No FCW candidate this tick", 30, 118, 0.44,
                  cv::Scalar(230, 230, 235), 1);
  }

  if (debug.has_runner_up_candidate) {
    drawCandidateCard(debug.runner_up_candidate, "Runner-Up", 18, 190,
                      kPanelWidth - 36, 88, false);
  } else {
    cv::rectangle(panel, cv::Rect(18, 190, kPanelWidth - 36, 60),
                  cv::Scalar(31, 31, 35), cv::FILLED);
    cv::rectangle(panel, cv::Rect(18, 190, kPanelWidth - 36, 60),
                  cv::Scalar(75, 75, 82), 1);
    drawPanelText("No runner-up candidate", 30, 225, 0.38,
                  cv::Scalar(190, 190, 195), 1);
  }

  drawSectionHeader("Why Is It Risky?", 290);
  if (debug.has_best_candidate) {
    const auto &cand = debug.best_candidate;
    drawBar(22, 308, kPanelWidth - 44, "Closing score", cand.closing_score,
            0.40f, cv::Scalar(70, 140, 255));
    drawBar(22, 334, kPanelWidth - 44, "Range score", cand.range_score, 0.24f,
            cv::Scalar(70, 200, 255));
    drawBar(22, 360, kPanelWidth - 44, "Quality score", cand.quality_score,
            0.21f, cv::Scalar(120, 220, 120));
    drawBar(22, 386, kPanelWidth - 44, "TTC score", cand.ttc_score, 0.10f,
            cv::Scalar(0, 215, 235));
    drawBar(22, 412, kPanelWidth - 44, "Physics score", cand.physics_score,
            0.05f, cv::Scalar(120, 120, 255));
    drawBar(22, 438, kPanelWidth - 44, "Final risk", cand.risk_score, 1.00f,
            fcwLevelColor(cand.active_level));
  }

  drawSectionHeader("Escalation + Geometry", 466);
  if (debug.has_best_candidate) {
    const auto &cand = debug.best_candidate;
    int badge_x = 22;
    int badge_y = 474;
    badge_x += drawBadge(badge_x, badge_y,
                         std::string("Base ") + adas::fcwRiskLevelName(cand.base_level),
                         true, fcwLevelColor(cand.base_level));
    badge_x += drawBadge(badge_x, badge_y, "TTC floor", cand.ttc_caution_floor,
                         cv::Scalar(0, 215, 235));
    badge_x += drawBadge(badge_x, badge_y, "Warn floor", cand.ttc_warn_floor,
                         cv::Scalar(0, 165, 255));
    badge_x += drawBadge(badge_x, badge_y, "Last ditch", cand.ttc_last_ditch,
                         cv::Scalar(0, 125, 255));
    badge_x = 22;
    badge_y += 24;
    badge_x += drawBadge(badge_x, badge_y, "Immediate warn",
                         cand.ttc_immediate_warn, cv::Scalar(0, 165, 255));
    badge_x += drawBadge(badge_x, badge_y, "Immediate critical",
                         cand.ttc_immediate_critical, cv::Scalar(0, 0, 255));
    badge_x += drawBadge(badge_x, badge_y, "Cam grace",
                         cand.camera_drop_grace_used,
                         cv::Scalar(180, 110, 255));
    badge_x += drawBadge(badge_x, badge_y, "Physics",
                         cand.physics_contributed, cv::Scalar(110, 110, 255));

    drawPathGauge(22, 530, 270, cand);
    drawStoppingGauge(318, 530, 270, cand);
  }

  drawSectionHeader("Rejected This Tick", 570);
  if (debug.rejected_candidates.empty()) {
    drawPanelText("No rejected objects", 22, 592, 0.34,
                  cv::Scalar(175, 175, 180), 1);
    return;
  }

  int y = 592;
  for (const auto &item : debug.rejected_candidates) {
    std::ostringstream ss;
    ss << "ID " << item.object_id << "  "
       << adas::fcwDropReasonName(item.drop_reason)
       << "  Z " << std::fixed << std::setprecision(1) << item.range_m
       << "m  TTC " << fmtFloat(item.ttc_s, "s")
       << "  V " << fmtFloat(item.velocity_mps, "m/s", 1);
    drawPanelText(ss.str(), 22, y, 0.33,
                  cv::Scalar(220, 205, 175), 1);
    y += 18;
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
    renderFcwReasoningPanel(panel, frame.fcw_debug_context);

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
