// File: include/adas/stage_e/BEVDashboardMath.hpp
// Pure helper math for BEV geometry and lifecycle timing.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace adas::bev {

inline float degToRad(float deg) {
  constexpr float kPi = 3.14159265358979323846f;
  return deg * kPi / 180.0f;
}

inline float angleFromPixel(float centroid_x_px, float c_x_px, float f_x_px) {
  if (f_x_px <= 1e-6f) {
    return 0.0f;
  }
  return std::atan((centroid_x_px - c_x_px) / f_x_px);
}

inline bool inAngleSpan(float angle_rad, float half_span_deg) {
  return std::abs(angle_rad) <= degToRad(half_span_deg);
}

inline float lateralFromRangeAndAngle(float range_m, float angle_rad) {
  return range_m * std::tan(angle_rad);
}

inline float ttlRemaining01(uint64_t now_ns, uint64_t last_update_ns,
                            uint32_t ttl_ms) {
  if (ttl_ms == 0) {
    return 0.0f;
  }
  if (last_update_ns == 0) {
    return 0.0f;
  }
  if (now_ns <= last_update_ns) {
    return 1.0f;
  }

  const uint64_t elapsed_ms = (now_ns - last_update_ns) / 1000000ULL;
  if (elapsed_ms >= ttl_ms) {
    return 0.0f;
  }
  const float remain_ms = static_cast<float>(ttl_ms - elapsed_ms);
  return remain_ms / static_cast<float>(ttl_ms);
}

inline bool shouldKeepTrack(uint64_t now_ns, uint64_t last_cam_update_ns,
                            uint64_t last_range_update_ns,
                            uint64_t ttc_hold_until_ns,
                            uint32_t dead_track_cleanup_ms) {
  if (now_ns < ttc_hold_until_ns) {
    return true;
  }

  const float cam_remain =
      ttlRemaining01(now_ns, last_cam_update_ns, dead_track_cleanup_ms);
  const float range_remain =
      ttlRemaining01(now_ns, last_range_update_ns, dead_track_cleanup_ms);
  return (cam_remain > 0.0f) || (range_remain > 0.0f);
}

} // namespace adas::bev
