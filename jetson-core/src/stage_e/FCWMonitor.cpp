// File: src/stage_e/FCWMonitor.cpp
// Forward Collision Warning with fused-first risk model.
#include "adas/stage_e/FCWMonitor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>

#include "adas/common/Globals.hpp"

namespace adas {

namespace {

constexpr float kGravityMps2 = 9.81f;
constexpr uint64_t kNsPerMs = 1000000ULL;
constexpr uint64_t kTrackForgetNs = 2000000000ULL;

float normalizedRangeScore(float range_m, float min_range_m, float max_range_m) {
  const float span = std::max(max_range_m - min_range_m, 0.1f);
  return std::clamp((max_range_m - range_m) / span, 0.0f, 1.0f);
}

float normalizedClosingScore(float closing_mps, float min_closing_mps) {
  if (closing_mps <= min_closing_mps) {
    return 0.0f;
  }
  return std::clamp((closing_mps - min_closing_mps) / 8.0f, 0.0f, 1.0f);
}

float normalizedQualityScore(float quality, float min_quality) {
  if (quality <= min_quality) {
    return 0.0f;
  }
  const float denom = std::max(1.0f - min_quality, 1e-3f);
  return std::clamp((quality - min_quality) / denom, 0.0f, 1.0f);
}

float normalizedTtcScore(float ttc_s, float threshold_s) {
  if (!std::isfinite(ttc_s) || ttc_s >= threshold_s || threshold_s <= 0.0f) {
    return 0.0f;
  }
  return std::clamp((threshold_s - ttc_s) / threshold_s, 0.0f, 1.0f);
}

} // namespace

FCWMonitor::FCWMonitor(const Config &config) : config_(config) {}

bool FCWMonitor::isRelevantClass(int cls) {
  // COCO classes relevant for FCW:
  // 0 = person, 1 = bicycle, 2 = car, 3 = motorcycle, 5 = bus, 7 = truck
  switch (cls) {
  case 0:
  case 1:
  case 2:
  case 3:
  case 5:
  case 7:
    return true;
  default:
    return false;
  }
}

float FCWMonitor::calculateStoppingDistance() const {
  if (ego_velocity_mps_ <= 0.0f) {
    return 0.0f;
  }

  // Physics formula: stopping distance = v^2 / (2 * mu * g) + v * t_reaction.
  const float braking_distance =
      (ego_velocity_mps_ * ego_velocity_mps_) /
      (2.0f * config_.friction_coefficient * kGravityMps2);
  const float reaction_distance = ego_velocity_mps_ * config_.reaction_time_s;
  return braking_distance + reaction_distance;
}

float FCWMonitor::pathHalfWidth(float range_m) const {
  const float r = std::max(range_m, 0.0f);
  return config_.path_half_width_m + config_.path_width_growth_per_m * r;
}

FCWMonitor::RiskLevel FCWMonitor::classifyRisk(float risk_score) const {
  if (risk_score >= config_.critical_risk_threshold) {
    return RiskLevel::Critical;
  }
  if (risk_score >= config_.warn_risk_threshold) {
    return RiskLevel::Warn;
  }
  if (risk_score >= config_.caution_risk_threshold) {
    return RiskLevel::Caution;
  }
  return RiskLevel::Safe;
}

uint32_t FCWMonitor::dwellForLevel(const Config &config, RiskLevel level) {
  switch (level) {
  case RiskLevel::Critical:
    return config.critical_dwell_ms;
  case RiskLevel::Warn:
    return config.warn_dwell_ms;
  case RiskLevel::Caution:
    return config.caution_dwell_ms;
  case RiskLevel::Safe:
  default:
    return config.clear_dwell_ms;
  }
}

FCWMonitor::RiskLevel FCWMonitor::applyDwell(TrackState &state,
                                             RiskLevel desired_level,
                                             uint64_t now_ns) const {
  if (desired_level == state.level) {
    state.pending_level = desired_level;
    state.pending_since_ns = now_ns;
    return state.level;
  }

  if (state.pending_level != desired_level) {
    state.pending_level = desired_level;
    state.pending_since_ns = now_ns;
    return state.level;
  }

  const uint64_t elapsed_ms =
      (now_ns > state.pending_since_ns) ? (now_ns - state.pending_since_ns) / kNsPerMs : 0ULL;
  const uint32_t dwell_ms = dwellForLevel(config_, desired_level);
  if (elapsed_ms >= dwell_ms) {
    state.level = desired_level;
    state.pending_level = desired_level;
    state.pending_since_ns = now_ns;
  }
  return state.level;
}

std::optional<FCWAlert>
FCWMonitor::check(const std::vector<FusedObject> &objects,
                  uint64_t current_time_ns) {
  struct Candidate {
    const FusedObject *obj = nullptr;
    RiskLevel level = RiskLevel::Safe;
    float risk = 0.0f;
    bool physics_contrib = false;
    bool ttc_last_ditch = false;
  };

  Candidate best;
  std::unordered_set<uint64_t> seen_ids;

  float stopping_distance_m = 0.0f;
  if (config_.use_physics_fcw && ego_velocity_mps_ > 0.5f) {
    stopping_distance_m = calculateStoppingDistance();
  }

  for (const auto &obj : objects) {
    if (obj.object_id == UINT64_MAX) {
      continue;
    }
    seen_ids.insert(obj.object_id);
    TrackState &state = track_states_[obj.object_id];
    state.last_seen_ns = current_time_ns;

    const auto demote_to_safe = [&]() {
      state.last_risk = 0.0f;
      applyDwell(state, RiskLevel::Safe, current_time_ns);
    };

    if (!obj.has_radar || !obj.speed_fresh) {
      demote_to_safe();
      continue;
    }
    if (!isRelevantClass(obj.object_class)) {
      demote_to_safe();
      continue;
    }
    if (obj.range_m < config_.min_range_m || obj.range_m > config_.max_range_m) {
      demote_to_safe();
      continue;
    }
    if (obj.fusion_quality < config_.min_fusion_quality) {
      demote_to_safe();
      continue;
    }

    const float closing_mps = obj.radial_vel_mps; // positive=inward/toward
    if (closing_mps <= config_.min_closing_speed_mps) {
      demote_to_safe();
      continue;
    }

    const float lane_half_width_m = pathHalfWidth(obj.range_m);
    const bool in_path = std::abs(obj.x_lateral_m) <= lane_half_width_m;
    if (!in_path) {
      // Ignore out-of-path objects to suppress adjacent-lane triggers.
      demote_to_safe();
      continue;
    }

    const float range_score =
        normalizedRangeScore(obj.range_m, config_.min_range_m, config_.max_range_m);
    const float closing_score =
        normalizedClosingScore(closing_mps, config_.min_closing_speed_mps);
    const float quality_score =
        normalizedQualityScore(obj.fusion_quality, config_.min_fusion_quality);
    const float ttc_score = normalizedTtcScore(obj.ttc_s, config_.ttc_threshold_s);

    float physics_score = 0.0f;
    bool physics_contrib = false;
    if (config_.use_physics_fcw && stopping_distance_m > 0.0f &&
        obj.range_m < stopping_distance_m) {
      physics_score = std::clamp((stopping_distance_m - obj.range_m) /
                                     std::max(stopping_distance_m, 1.0f),
                                 0.0f, 1.0f);
      physics_contrib = true;
    }

    // Fused risk model: closeness + closing speed + fusion confidence.
    // TTC contributes lightly and serves as last-ditch escalation.
    float risk_score = 0.40f * closing_score + 0.24f * range_score +
                       0.21f * quality_score + 0.10f * ttc_score +
                       0.05f * physics_score;

    bool ttc_last_ditch = false;
    if (std::isfinite(obj.ttc_s) && obj.ttc_s <= config_.ttc_last_ditch_s) {
      risk_score = std::max(risk_score, config_.warn_risk_threshold + 0.02f);
      ttc_last_ditch = true;
    }

    const RiskLevel desired_level = classifyRisk(risk_score);
    state.last_risk = risk_score;
    const RiskLevel active_level = applyDwell(state, desired_level, current_time_ns);

    std::cout << "[StageE: 4_FCW] ID " << obj.object_id << " | Z: " << obj.range_m
              << "m | V: " << obj.radial_vel_mps << "m/s | TTC: " << obj.ttc_s
              << "s | Q: " << obj.fusion_quality << " | X: " << obj.x_lateral_m
              << "m -> risk " << risk_score << " level "
              << static_cast<int>(active_level)
              << (ttc_last_ditch ? " [TTC_LAST_DITCH]" : "") << "\n";

    if (active_level < RiskLevel::Warn) {
      continue;
    }

    if (best.obj == nullptr || active_level > best.level ||
        (active_level == best.level && risk_score > best.risk) ||
        (active_level == best.level && std::abs(risk_score - best.risk) < 1e-4f &&
         obj.ttc_s < best.obj->ttc_s)) {
      best.obj = &obj;
      best.level = active_level;
      best.risk = risk_score;
      best.physics_contrib = physics_contrib;
      best.ttc_last_ditch = ttc_last_ditch;
    }
  }

  // Garbage-collect stale per-object risk states.
  for (auto it = track_states_.begin(); it != track_states_.end();) {
    if (seen_ids.find(it->first) == seen_ids.end() &&
        (current_time_ns > it->second.last_seen_ns) &&
        (current_time_ns - it->second.last_seen_ns > kTrackForgetNs)) {
      it = track_states_.erase(it);
    } else {
      ++it;
    }
  }

  if (best.obj == nullptr) {
    return std::nullopt;
  }

  FCWAlert alert;
  alert.ttc_s = best.obj->ttc_s;
  alert.range_m = best.obj->range_m;
  alert.velocity_mps = best.obj->radial_vel_mps;
  alert.object_class = best.obj->object_class;
  alert.object_id = best.obj->object_id;
  alert.timestamp_ns = current_time_ns;
  alert.physics_triggered = best.physics_contrib;

  if (g_verbose_mode.load()) {
    std::cout << "[FCW] ALERT: id=" << alert.object_id << " TTC=" << alert.ttc_s
              << "s range=" << alert.range_m << "m v=" << alert.velocity_mps
              << "m/s level=" << static_cast<int>(best.level)
              << (best.ttc_last_ditch ? " [TTC_LAST_DITCH]" : "")
              << "\n";
  }

  return alert;
}

} // namespace adas
