// File: include/adas/stage_e/FCWMonitor.hpp
// Forward Collision Warning monitor
#pragma once

#include "adas/stage_e/SensorFusion.hpp"

#include <optional>
#include <vector>

namespace adas {

/// FCW Alert data
struct FCWAlert {
    float ttc_s;            // Time-to-collision
    float range_m;          // Distance to threat
    float velocity_mps;     // Closing velocity
    int object_class;       // Object type (person, car, etc.)
    uint64_t timestamp_ns;  // When alert was generated
    bool physics_triggered; // True if physics-based FCW triggered this alert

    FCWAlert()
        : ttc_s(0), range_m(0), velocity_mps(0), object_class(0), timestamp_ns(0),
          physics_triggered(false) {}
};

/// FCWMonitor: Checks fused objects for collision threats
class FCWMonitor {
  public:
    struct Config {
        float ttc_threshold_s; // Alert if TTC below this (FR20)
        float min_range_m;     // Ignore if closer than this (already hit)
        float max_range_m;     // Ignore if too far

        // Physics-based FCW parameters
        float friction_coefficient; // Road friction (0.7 = dry, 0.4 = wet, 0.2 = ice)
        float reaction_time_s;      // Driver reaction time before braking
        bool use_physics_fcw;       // Enable physics-based calculation

        Config()
            : ttc_threshold_s(3.0f), min_range_m(0.5f), max_range_m(50.0f),
              friction_coefficient(0.7f), reaction_time_s(2.5f), use_physics_fcw(false) {}
    };

    explicit FCWMonitor(const Config &config = Config());

    /// Set ego vehicle forward velocity (for physics-based FCW)
    /// @param velocity_mps Ego vehicle speed in m/s (positive = forward)
    void setEgoVelocity(float velocity_mps) { ego_velocity_mps_ = velocity_mps; }

    /// Get current ego velocity
    float getEgoVelocity() const { return ego_velocity_mps_; }

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

  private:
    Config config_;
    float ego_velocity_mps_ = 0.0f;
};

} // namespace adas
