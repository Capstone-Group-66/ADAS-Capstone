// File: src/stage_e/FCWMonitor.cpp
// Forward Collision Warning with fused-first risk model.
#include "adas/stage_e/FCWMonitor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <unordered_set>

#include "adas/common/Globals.hpp"

namespace adas {

namespace {

constexpr float kGravityMps2 = 9.81f;
constexpr uint64_t kNsPerMs = 1000000ULL;
constexpr uint64_t kTrackForgetNs = 2000000000ULL;

float normalizedRangeScore(float range_m, float min_range_m,
                           float max_range_m) {
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

uint8_t levelRank(FCWMonitor::RiskLevel level) {
  return static_cast<uint8_t>(level);
}

template <typename T> const T &minByTtc(const T &a, const T &b) {
  const bool a_valid = std::isfinite(a.ttc_s) && a.ttc_s > 0.0f;
  const bool b_valid = std::isfinite(b.ttc_s) && b.ttc_s > 0.0f;
  if (a_valid && b_valid) {
    return (a.ttc_s <= b.ttc_s) ? a : b;
  }
  if (a_valid) {
    return a;
  }
  return b;
}

std::string runnerUpReason(const FCWDebugCandidate &best,
                           const FCWDebugCandidate &runner_up) {
  if (runner_up.active_level < best.active_level) {
    return "lower level";
  }
  if (runner_up.desired_level < best.desired_level) {
    return "weaker escalation";
  }
  if (std::abs(runner_up.risk_score - best.risk_score) > 1e-4f) {
    return "lower risk";
  }
  const bool best_ttc_valid = std::isfinite(best.ttc_s) && best.ttc_s > 0.0f;
  const bool runner_ttc_valid =
      std::isfinite(runner_up.ttc_s) && runner_up.ttc_s > 0.0f;
  if (best_ttc_valid && runner_ttc_valid && runner_up.ttc_s > best.ttc_s) {
    return "higher TTC";
  }
  return "lower priority";
}

} // namespace

FCWMonitor::FCWMonitor(const Config &config)
    : config_(config), min_trigger_object_speed_mps_(std::max(
                           0.0f, config.min_trigger_object_speed_mps)) {}

void FCWMonitor::setMinTriggerObjectSpeedGateMps(float min_speed_mps) {
  if (!std::isfinite(min_speed_mps) || min_speed_mps < 0.0f) {
    min_speed_mps = 0.0f;
  }
  min_trigger_object_speed_mps_.store(min_speed_mps, std::memory_order_relaxed);
}

float FCWMonitor::getMinTriggerObjectSpeedGateMps() const {
  return min_trigger_object_speed_mps_.load(std::memory_order_relaxed);
}

bool FCWMonitor::isRelevantClass(int cls) {
  // Canonical front-camera classes relevant for FCW:
  // 0 = Car, 1 = Bicycle, 2 = Person, 3 = RoadSign
  switch (cls) {
  case 0:
  case 1:
  case 2:
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

  const uint64_t elapsed_ms = (now_ns > state.pending_since_ns)
                                  ? (now_ns - state.pending_since_ns) / kNsPerMs
                                  : 0ULL;
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
  struct CandidateEval {
    const FusedObject *obj = nullptr;
    RiskLevel base_level = RiskLevel::Safe;
    RiskLevel desired_level = RiskLevel::Safe;
    RiskLevel active_level = RiskLevel::Safe;
    float base_risk = 0.0f;
    float risk = 0.0f;
    float range_score = 0.0f;
    float closing_score = 0.0f;
    float quality_score = 0.0f;
    float ttc_score = 0.0f;
    float physics_score = 0.0f;
    float lane_half_width_m = 0.0f;
    float stopping_distance_m = 0.0f;
    bool physics_contrib = false;
    bool camera_drop_grace = false;
    bool ttc_caution_floor = false;
    bool ttc_warn_floor = false;
    bool ttc_last_ditch = false;
    bool ttc_immediate_warn = false;
    bool ttc_immediate_critical = false;
    bool gate_camera_ok = false;
    bool gate_has_radar = false;
    bool gate_speed_fresh = false;
    bool gate_class_relevant = false;
    bool gate_range_ok = false;
    bool gate_quality_ok = false;
    bool gate_closing_ok = false;
    bool gate_in_path = false;

    FCWDebugCandidate toDebugCandidate() const {
      FCWDebugCandidate debug;
      if (!obj) {
        return debug;
      }
      debug.valid = true;
      debug.object_id = obj->object_id;
      debug.base_level = static_cast<uint8_t>(base_level);
      debug.desired_level = static_cast<uint8_t>(desired_level);
      debug.active_level = static_cast<uint8_t>(active_level);
      debug.base_risk_score = base_risk;
      debug.risk_score = risk;
      debug.ttc_s = obj->ttc_s;
      debug.range_m = obj->range_m;
      debug.velocity_mps = obj->radial_vel_mps;
      debug.x_lateral_m = obj->x_lateral_m;
      debug.lane_half_width_m = lane_half_width_m;
      debug.fusion_quality = obj->fusion_quality;
      debug.range_score = range_score;
      debug.closing_score = closing_score;
      debug.quality_score = quality_score;
      debug.ttc_score = ttc_score;
      debug.physics_score = physics_score;
      debug.stopping_distance_m = stopping_distance_m;
      debug.camera_drop_grace_used = camera_drop_grace;
      debug.physics_contributed = physics_contrib;
      debug.ttc_caution_floor = ttc_caution_floor;
      debug.ttc_warn_floor = ttc_warn_floor;
      debug.ttc_last_ditch = ttc_last_ditch;
      debug.ttc_immediate_warn = ttc_immediate_warn;
      debug.ttc_immediate_critical = ttc_immediate_critical;
      debug.gate_camera_ok = gate_camera_ok;
      debug.gate_has_radar = gate_has_radar;
      debug.gate_speed_fresh = gate_speed_fresh;
      debug.gate_class_relevant = gate_class_relevant;
      debug.gate_range_ok = gate_range_ok;
      debug.gate_quality_ok = gate_quality_ok;
      debug.gate_closing_ok = gate_closing_ok;
      debug.gate_in_path = gate_in_path;
      return debug;
    }
  };

  CandidateEval best_alert;
  std::vector<CandidateEval> candidate_evals;
  std::vector<FCWDebugRejected> rejected;
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
    bool invalid_demote_grace_used = false;
    bool invalid_state_hold_used = false;

    const auto addRejected = [&](FCWDropReason reason) {
      FCWDebugRejected item;
      item.object_id = obj.object_id;
      item.drop_reason = reason;
      item.ttc_s = obj.ttc_s;
      item.range_m = obj.range_m;
      item.velocity_mps = obj.radial_vel_mps;
      item.x_lateral_m = obj.x_lateral_m;
      item.fusion_quality = obj.fusion_quality;
      rejected.push_back(item);
    };

    const auto demote_to_safe = [&](const char *reason) {
      state.last_risk = 0.0f;
      const uint64_t valid_age_ms =
          (state.last_valid_ns > 0 && current_time_ns > state.last_valid_ns)
              ? (current_time_ns - state.last_valid_ns) / kNsPerMs
              : 0ULL;

      const bool has_active_state_hold =
          config_.invalid_state_hold_ms > 0 && state.last_valid_ns > 0 &&
          levelRank(state.level) > levelRank(RiskLevel::Safe) &&
          valid_age_ms <= config_.invalid_state_hold_ms;
      if (has_active_state_hold) {
        invalid_state_hold_used = true;
        // Keep active level pinned during short invalid gaps.
        state.pending_level = state.level;
        state.pending_since_ns = current_time_ns;
      } else {
        const bool has_pending_escalation =
            config_.invalid_demote_grace_ms > 0 &&
            state.level == RiskLevel::Safe &&
            state.pending_level != RiskLevel::Safe &&
            state.pending_since_ns > 0;
        if (has_pending_escalation) {
          const uint64_t pending_age_ms =
              (current_time_ns > state.pending_since_ns)
                  ? (current_time_ns - state.pending_since_ns) / kNsPerMs
                  : 0ULL;
          if (pending_age_ms <= config_.invalid_demote_grace_ms) {
            invalid_demote_grace_used = true;
          } else {
            applyDwell(state, RiskLevel::Safe, current_time_ns);
          }
        } else {
          applyDwell(state, RiskLevel::Safe, current_time_ns);
        }
      }

      if (config_.log_fcw_drop_reasons && g_verbose_mode.load()) {
        std::cout << "[StageE: 4_FCW_DROP] ID " << obj.object_id
                  << " | reason: " << reason << " | active "
                  << static_cast<int>(state.level) << " (desired 0, pending "
                  << static_cast<int>(state.pending_level) << ")"
                  << " | valid_age_ms: " << valid_age_ms
                  << (invalid_state_hold_used ? " [INVALID_STATE_HOLD]" : "")
                  << (invalid_demote_grace_used ? " [INVALID_DEMOTE_GRACE]"
                                                : "")
                  << "\n";
      }
    };

    const bool camera_fresh = obj.camera_age_ms <= config_.camera_hold_ms;
    bool camera_drop_grace = false;
    if (!camera_fresh) {
      const bool has_camera_history = (obj.sources & SRC_CAM_F) != 0;
      const bool radar_recent_for_grace =
          obj.radar_age_ms <= config_.camera_drop_radar_recent_ms;
      const float min_grace_quality =
          std::max(config_.min_fusion_quality, config_.camera_drop_min_quality);
      camera_drop_grace =
          has_camera_history &&
          obj.camera_age_ms <= config_.camera_drop_track_hold_ms &&
          radar_recent_for_grace && obj.fusion_quality >= min_grace_quality;
      if (!camera_drop_grace) {
        addRejected(FCWDropReason::CamAge);
        demote_to_safe("CAM_AGE");
        continue;
      }
    }

    const bool gate_camera_ok = camera_fresh || camera_drop_grace;
    const bool gate_has_radar = obj.has_radar;
    const bool gate_speed_fresh = obj.speed_fresh;
    const bool gate_class_relevant = isRelevantClass(obj.object_class);
    const bool gate_range_ok = obj.range_m >= config_.min_range_m &&
                               obj.range_m <= config_.max_range_m;
    const bool gate_quality_ok =
        obj.fusion_quality >= config_.min_fusion_quality;
    const float closing_mps = obj.radial_vel_mps; // positive=inward/toward
    const bool gate_closing_ok = closing_mps > config_.min_closing_speed_mps;
    const float lane_half_width_m = pathHalfWidth(obj.range_m);
    const bool gate_in_path = std::abs(obj.x_lateral_m) <= lane_half_width_m;

    if (!obj.has_radar) {
      addRejected(FCWDropReason::NoRadar);
      demote_to_safe("NO_RADAR");
      continue;
    }
    if (!obj.speed_fresh) {
      addRejected(FCWDropReason::SpeedStale);
      demote_to_safe("SPEED_STALE");
      continue;
    }
    if (!isRelevantClass(obj.object_class)) {
      addRejected(FCWDropReason::ClassFilter);
      demote_to_safe("CLASS_FILTER");
      continue;
    }
    if (obj.range_m < config_.min_range_m ||
        obj.range_m > config_.max_range_m) {
      addRejected(FCWDropReason::RangeGate);
      demote_to_safe("RANGE_GATE");
      continue;
    }
    if (obj.fusion_quality < config_.min_fusion_quality) {
      addRejected(FCWDropReason::LowQuality);
      demote_to_safe("LOW_QUALITY");
      continue;
    }
    if (closing_mps <= config_.min_closing_speed_mps) {
      addRejected(FCWDropReason::LowClosingSpeed);
      demote_to_safe("LOW_CLOSING_SPEED");
      continue;
    }
    const bool in_path = gate_in_path;
    if (!in_path) {
      // Ignore out-of-path objects to suppress adjacent-lane triggers.
      addRejected(FCWDropReason::OutOfPath);
      demote_to_safe("OUT_OF_PATH");
      continue;
    }

    state.last_valid_ns = current_time_ns;

    const float range_score = normalizedRangeScore(
        obj.range_m, config_.min_range_m, config_.max_range_m);
    const float closing_score =
        normalizedClosingScore(closing_mps, config_.min_closing_speed_mps);
    const float quality_score =
        normalizedQualityScore(obj.fusion_quality, config_.min_fusion_quality);
    const float ttc_score =
        normalizedTtcScore(obj.ttc_s, config_.ttc_threshold_s);

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
    const float base_risk_score = 0.40f * closing_score + 0.24f * range_score +
                                  0.21f * quality_score + 0.10f * ttc_score +
                                  0.05f * physics_score;
    float risk_score = base_risk_score;

    bool ttc_caution_floor = false;
    bool ttc_warn_floor = false;
    const bool ttc_valid = std::isfinite(obj.ttc_s) && obj.ttc_s > 0.0f;
    if (ttc_valid && obj.ttc_s <= config_.ttc_threshold_s) {
      risk_score = std::max(risk_score, config_.caution_risk_threshold + 0.01f);
      ttc_caution_floor = true;
    }
    // If TTC is very short and the object is close/in-path, force at least
    // WARN.
    const float ttc_warn_floor_s =
        std::max(0.6f, 0.8f * config_.ttc_threshold_s);
    if (ttc_valid && obj.ttc_s <= ttc_warn_floor_s && obj.range_m <= 5.0f) {
      risk_score = std::max(risk_score, config_.warn_risk_threshold + 0.01f);
      ttc_warn_floor = true;
    }

    bool ttc_last_ditch = false;
    if (ttc_valid && obj.ttc_s <= config_.ttc_last_ditch_s) {
      risk_score = std::max(risk_score, config_.warn_risk_threshold + 0.02f);
      ttc_last_ditch = true;
    }

    bool ttc_immediate_warn = false;
    bool ttc_immediate_critical = false;
    const RiskLevel base_level = classifyRisk(base_risk_score);
    RiskLevel desired_level = classifyRisk(risk_score);
    if (ttc_valid && obj.ttc_s <= config_.ttc_immediate_critical_s) {
      if (levelRank(desired_level) < levelRank(RiskLevel::Critical)) {
        desired_level = RiskLevel::Critical;
      }
      ttc_immediate_critical = true;
    } else if (ttc_valid && obj.ttc_s <= config_.ttc_immediate_warn_s &&
               levelRank(desired_level) < levelRank(RiskLevel::Warn)) {
      desired_level = RiskLevel::Warn;
      ttc_immediate_warn = true;
    }

    bool camera_drop_hold_level = false;
    if (camera_drop_grace &&
        levelRank(state.level) > levelRank(RiskLevel::Safe) &&
        levelRank(desired_level) < levelRank(state.level)) {
      desired_level = state.level;
      camera_drop_hold_level = true;
    }

    const bool bypass_escalation_dwell =
        (ttc_immediate_warn || ttc_immediate_critical) &&
        levelRank(desired_level) > levelRank(state.level);
    state.last_risk = risk_score;
    RiskLevel active_level = RiskLevel::Safe;
    if (bypass_escalation_dwell) {
      state.level = desired_level;
      state.pending_level = desired_level;
      state.pending_since_ns = current_time_ns;
      active_level = state.level;
    } else {
      active_level = applyDwell(state, desired_level, current_time_ns);
    }

    std::cout << "[StageE: 4_FCW] ID " << obj.object_id
              << " | Z: " << obj.range_m << "m | V: " << obj.radial_vel_mps
              << "m/s | TTC: " << obj.ttc_s << "s | Q: " << obj.fusion_quality
              << " | X: " << obj.x_lateral_m << "m -> risk " << risk_score
              << " level " << static_cast<int>(active_level) << " (desired "
              << static_cast<int>(desired_level) << ", pending "
              << static_cast<int>(state.pending_level) << ")"
              << (camera_drop_grace ? " [CAM_DROP_GRACE]" : "")
              << (camera_drop_hold_level ? " [CAM_DROP_HOLD_LEVEL]" : "")
              << (invalid_demote_grace_used ? " [INVALID_DEMOTE_GRACE]" : "")
              << (ttc_caution_floor ? " [TTC_CAUTION_FLOOR]" : "")
              << (ttc_warn_floor ? " [TTC_WARN_FLOOR]" : "")
              << (ttc_immediate_warn ? " [TTC_IMMEDIATE_WARN]" : "")
              << (ttc_immediate_critical ? " [TTC_IMMEDIATE_CRITICAL]" : "")
              << (ttc_last_ditch ? " [TTC_LAST_DITCH]" : "") << "\n";

    CandidateEval eval;
    eval.obj = &obj;
    eval.base_level = base_level;
    eval.desired_level = desired_level;
    eval.active_level = active_level;
    eval.base_risk = base_risk_score;
    eval.risk = risk_score;
    eval.range_score = range_score;
    eval.closing_score = closing_score;
    eval.quality_score = quality_score;
    eval.ttc_score = ttc_score;
    eval.physics_score = physics_score;
    eval.lane_half_width_m = lane_half_width_m;
    eval.stopping_distance_m = stopping_distance_m;
    eval.physics_contrib = physics_contrib;
    eval.camera_drop_grace = camera_drop_grace;
    eval.ttc_caution_floor = ttc_caution_floor;
    eval.ttc_warn_floor = ttc_warn_floor;
    eval.ttc_last_ditch = ttc_last_ditch;
    eval.ttc_immediate_warn = ttc_immediate_warn;
    eval.ttc_immediate_critical = ttc_immediate_critical;
    eval.gate_camera_ok = gate_camera_ok;
    eval.gate_has_radar = gate_has_radar;
    eval.gate_speed_fresh = gate_speed_fresh;
    eval.gate_class_relevant = gate_class_relevant;
    eval.gate_range_ok = gate_range_ok;
    eval.gate_quality_ok = gate_quality_ok;
    eval.gate_closing_ok = gate_closing_ok;
    eval.gate_in_path = gate_in_path;
    candidate_evals.push_back(eval);

    if (active_level < RiskLevel::Warn) {
      continue;
    }

    if (best_alert.obj == nullptr || active_level > best_alert.active_level ||
        (active_level == best_alert.active_level &&
         risk_score > best_alert.risk) ||
        (active_level == best_alert.active_level &&
         std::abs(risk_score - best_alert.risk) < 1e-4f &&
         minByTtc(obj, *best_alert.obj).object_id == obj.object_id)) {
      best_alert = eval;
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

  auto candidateComparator = [](const CandidateEval &a,
                                const CandidateEval &b) {
    if (a.active_level != b.active_level) {
      return a.active_level > b.active_level;
    }
    if (std::abs(a.risk - b.risk) >= 1e-4f) {
      return a.risk > b.risk;
    }
    const bool a_ttc_valid =
        a.obj && std::isfinite(a.obj->ttc_s) && a.obj->ttc_s > 0.0f;
    const bool b_ttc_valid =
        b.obj && std::isfinite(b.obj->ttc_s) && b.obj->ttc_s > 0.0f;
    if (a_ttc_valid != b_ttc_valid) {
      return a_ttc_valid;
    }
    if (a_ttc_valid && b_ttc_valid &&
        std::abs(a.obj->ttc_s - b.obj->ttc_s) >= 1e-4f) {
      return a.obj->ttc_s < b.obj->ttc_s;
    }
    return a.obj->object_id < b.obj->object_id;
  };
  std::sort(candidate_evals.begin(), candidate_evals.end(),
            candidateComparator);

  auto rejectedComparator = [](const FCWDebugRejected &a,
                               const FCWDebugRejected &b) {
    const float a_ttc =
        (std::isfinite(a.ttc_s) && a.ttc_s > 0.0f) ? a.ttc_s : 1e6f;
    const float b_ttc =
        (std::isfinite(b.ttc_s) && b.ttc_s > 0.0f) ? b.ttc_s : 1e6f;
    if (std::abs(a_ttc - b_ttc) >= 1e-4f) {
      return a_ttc < b_ttc;
    }
    if (std::abs(a.range_m - b.range_m) >= 1e-4f) {
      return a.range_m < b.range_m;
    }
    return std::abs(a.velocity_mps) > std::abs(b.velocity_mps);
  };
  std::sort(rejected.begin(), rejected.end(), rejectedComparator);
  if (rejected.size() > 3) {
    rejected.resize(3);
  }

  FCWDebugSnapshot snapshot;
  snapshot.timestamp_ns = current_time_ns;
  if (!candidate_evals.empty()) {
    snapshot.has_best_candidate = true;
    snapshot.best_candidate = candidate_evals.front().toDebugCandidate();
    if (candidate_evals.size() > 1) {
      snapshot.has_runner_up_candidate = true;
      snapshot.runner_up_candidate = candidate_evals[1].toDebugCandidate();
      snapshot.runner_up_candidate.comparison_reason =
          runnerUpReason(snapshot.best_candidate, snapshot.runner_up_candidate);
    }
  }
  snapshot.rejected_candidates = rejected;
  last_debug_snapshot_ = snapshot;

  if (!candidate_evals.empty()) {
    const auto &best_eval = candidate_evals.front();
    last_evaluation_.has_candidate = true;
    last_evaluation_.object_id = best_eval.obj->object_id;
    last_evaluation_.level = static_cast<uint8_t>(best_eval.active_level);
    last_evaluation_.risk_score = best_eval.risk;
    last_evaluation_.ttc_s = best_eval.obj->ttc_s;
    last_evaluation_.range_m = best_eval.obj->range_m;
    last_evaluation_.velocity_mps = best_eval.obj->radial_vel_mps;
    last_evaluation_.used_camera_drop_grace = best_eval.camera_drop_grace;
  } else {
    last_evaluation_ = FCWEvaluation();
  }

  if (best_alert.obj == nullptr) {
    return std::nullopt;
  }

  const float min_trigger_speed_mps =
      min_trigger_object_speed_mps_.load(std::memory_order_relaxed);
  if (best_alert.obj->radial_vel_mps < min_trigger_speed_mps) {
    if (g_verbose_mode.load()) {
      std::cout << "[FCW] SUPPRESSED by speed gate: id="
                << best_alert.obj->object_id
                << " v=" << best_alert.obj->radial_vel_mps
                << "m/s gate=" << min_trigger_speed_mps << "m/s\n";
    }
    return std::nullopt;
  }

  FCWAlert alert;
  alert.ttc_s = best_alert.obj->ttc_s;
  alert.range_m = best_alert.obj->range_m;
  alert.velocity_mps = best_alert.obj->radial_vel_mps;
  alert.object_class = best_alert.obj->object_class;
  alert.object_id = best_alert.obj->object_id;
  alert.timestamp_ns = current_time_ns;
  alert.physics_triggered = best_alert.physics_contrib;

  if (g_verbose_mode.load()) {
    std::cout << "[FCW] ALERT: id=" << alert.object_id << " TTC=" << alert.ttc_s
              << "s range=" << alert.range_m << "m v=" << alert.velocity_mps
              << "m/s level=" << static_cast<int>(best_alert.active_level)
              << (best_alert.camera_drop_grace ? " [CAM_DROP_GRACE]" : "")
              << (best_alert.ttc_last_ditch ? " [TTC_LAST_DITCH]" : "") << "\n";
  }

  return alert;
}

} // namespace adas
