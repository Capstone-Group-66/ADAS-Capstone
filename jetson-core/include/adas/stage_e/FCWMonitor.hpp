// File: include/adas/stage_e/FCWMonitor.hpp
// Forward Collision Warning monitor
#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "adas/stage_e/SensorFusion.hpp"

namespace adas {

/// FCW Alert data
struct FCWAlert {
    float ttc_s;            // Time-to-collision
    float range_m;          // Distance to threat
    float velocity_mps;     // Closing velocity
    int object_class;       // Object type (person, car, etc.)
    uint64_t object_id;     // Track ID for the triggering object
    uint64_t timestamp_ns;  // When alert was generated
    bool physics_triggered; // True if physics-based FCW triggered this alert

    FCWAlert()
        : ttc_s(0), range_m(0), velocity_mps(0), object_class(0),
          object_id(UINT64_MAX), timestamp_ns(0), physics_triggered(false) {}
};

/// FCW evaluation snapshot for BEV/debug visibility.
struct FCWEvaluation {
    bool has_candidate;          // False when no eligible object this tick
    uint64_t object_id;          // Best-evaluated object ID
    uint8_t level;               // FCWMonitor::RiskLevel cast to uint8_t
    float risk_score;            // [0..1] fused risk score
    float ttc_s;                 // Candidate TTC
    float range_m;               // Candidate range
    float velocity_mps;          // Candidate closing speed
    bool used_camera_drop_grace; // True when camera freshness grace was used

    FCWEvaluation()
        : has_candidate(false), object_id(UINT64_MAX), level(0), risk_score(0.0f),
          ttc_s(0.0f), range_m(0.0f), velocity_mps(0.0f),
          used_camera_drop_grace(false) {}
};

/// FCWMonitor: Checks fused objects for collision threats
class FCWMonitor {
  public:
    enum class RiskLevel : uint8_t { Safe = 0, Caution = 1, Warn = 2, Critical = 3 };

    struct Config {
        float ttc_threshold_s; // Alert if TTC below this (FR20)
        float min_range_m;     // Ignore if closer than this (already hit)
        float max_range_m;     // Ignore if too far
        float min_closing_speed_mps; // Positive=toward/inward
        float min_trigger_object_speed_mps; // Final alert gate (0=disabled)

        // Physics-based FCW parameters
        float friction_coefficient; // Road friction (0.7 = dry, 0.4 = wet, 0.2
                                    // = ice)
        float reaction_time_s;      // Driver reaction time before braking
        bool use_physics_fcw;       // Enable physics-based calculation

        // Fusion/path quality gates
        float min_fusion_quality;      // Reject weak radar-camera pairings
        float path_half_width_m;       // In-path lateral half-width at 0m
        float path_width_growth_per_m; // In-path width growth vs range

        // Risk thresholds
        float caution_risk_threshold;
        float warn_risk_threshold;
        float critical_risk_threshold;
        float ttc_last_ditch_s; // TTC escalation threshold, last-resort only
        float ttc_immediate_warn_s; // Immediate WARN escalation threshold
        float ttc_immediate_critical_s; // Immediate CRITICAL escalation threshold
        uint32_t camera_hold_ms; // FCW eligibility requires camera freshness
        uint32_t camera_drop_track_hold_ms; // Temporary radar-only continuation
        uint32_t camera_drop_radar_recent_ms; // Radar freshness for grace
        float camera_drop_min_quality; // Minimum fusion quality during grace
        uint32_t invalid_demote_grace_ms; // Keep pending escalation briefly
        uint32_t invalid_state_hold_ms; // Preserve active state across brief invalid ticks
        bool log_fcw_drop_reasons; // Emit explicit per-reason demotion logs

        // Dwell / hysteresis timing
        uint32_t caution_dwell_ms;
        uint32_t warn_dwell_ms;
        uint32_t critical_dwell_ms;
        uint32_t clear_dwell_ms;

        Config()
            : ttc_threshold_s(3.0f), min_range_m(0.5f), max_range_m(50.0f),
              min_closing_speed_mps(0.35f),
              min_trigger_object_speed_mps(0.0f),
              friction_coefficient(0.7f),
              reaction_time_s(2.5f), use_physics_fcw(false),
              min_fusion_quality(0.22f), path_half_width_m(0.85f),
              path_width_growth_per_m(0.035f), caution_risk_threshold(0.38f),
              warn_risk_threshold(0.56f), critical_risk_threshold(0.74f),
              ttc_last_ditch_s(0.85f), ttc_immediate_warn_s(2.8f),
              ttc_immediate_critical_s(1.2f), camera_hold_ms(400),
              camera_drop_track_hold_ms(1200), camera_drop_radar_recent_ms(150),
              camera_drop_min_quality(0.32f), invalid_demote_grace_ms(350),
              invalid_state_hold_ms(250), log_fcw_drop_reasons(false),
              caution_dwell_ms(180),
              warn_dwell_ms(140), critical_dwell_ms(80), clear_dwell_ms(220) {}
    };

    explicit FCWMonitor(const Config &config = Config());

    /// Set ego vehicle forward velocity (for physics-based FCW)
    /// @param velocity_mps Ego vehicle speed in m/s (positive = forward)
    void setEgoVelocity(float velocity_mps) { ego_velocity_mps_ = velocity_mps; }

    /// Get current ego velocity
    float getEgoVelocity() const { return ego_velocity_mps_; }

    /// Set final FCW trigger speed gate for object radial speed.
    /// @param min_speed_mps Minimum object radial speed in m/s.
    ///                      0 disables this final trigger gate.
    void setMinTriggerObjectSpeedGateMps(float min_speed_mps);

    /// Get current final FCW trigger speed gate in m/s.
    float getMinTriggerObjectSpeedGateMps() const;

    /// Check fused objects for FCW threats
    /// @param objects Fused objects from SensorFusion
    /// @return Alert if threat detected, empty otherwise
    std::optional<FCWAlert> check(const std::vector<FusedObject> &objects,
                                  uint64_t current_time_ns);

    /// Check if a specific object class is relevant for FCW
    /// @param cls Object class ID
    /// @return True if this class should trigger FCW (vehicles, people)
    static bool isRelevantClass(int cls);

    /// Get the current TTC threshold
    float getThreshold() const { return config_.ttc_threshold_s; }

    /// Calculate stopping distance at current ego velocity
    /// @return Stopping distance in meters (reaction + braking)
    float calculateStoppingDistance() const;

    /// Get last StageE-4 evaluation snapshot (used by BEV debug UI).
    FCWEvaluation getLastEvaluation() const { return last_evaluation_; }

  private:
    struct TrackState {
        RiskLevel level = RiskLevel::Safe;
        RiskLevel pending_level = RiskLevel::Safe;
        uint64_t pending_since_ns = 0;
        uint64_t last_seen_ns = 0;
        uint64_t last_valid_ns = 0;
        float last_risk = 0.0f;
    };

    float pathHalfWidth(float range_m) const;
    RiskLevel classifyRisk(float risk_score) const;
    RiskLevel applyDwell(TrackState &state, RiskLevel desired_level,
                         uint64_t now_ns) const;
    static uint32_t dwellForLevel(const Config &config, RiskLevel level);

    Config config_;
    float ego_velocity_mps_ = 0.0f;
    std::atomic<float> min_trigger_object_speed_mps_{0.0f};
    std::unordered_map<uint64_t, TrackState> track_states_;
    FCWEvaluation last_evaluation_;
};

} // namespace adas
