// File: src/stage_e/FCWMonitor.cpp
// Forward Collision Warning implementation with physics-based FCW
#include "adas/stage_e/FCWMonitor.hpp"

#include <cmath>
#include <iostream>

#include "adas/common/Globals.hpp"

namespace adas {

FCWMonitor::FCWMonitor(const Config &config) : config_(config) {}

bool FCWMonitor::isRelevantClass(int cls) {
  // COCO classes relevant for FCW:
  // 0 = person, 1 = bicycle, 2 = car, 3 = motorcycle,
  // 5 = bus, 7 = truck
  switch (cls) {
  case 0: // person
  case 1: // bicycle
  case 2: // car
  case 3: // motorcycle
  case 5: // bus
  case 7: // truck
    return true;
  default:
    return false;
  }
}

float FCWMonitor::calculateStoppingDistance() const {
  if (ego_velocity_mps_ <= 0.0f) {
    return 0.0f;
  }

  // Physics formula: stopping distance = v² / (2 * μ * g)
  // where μ = friction coefficient, g = 9.81 m/s²
  float braking_distance = (ego_velocity_mps_ * ego_velocity_mps_) /
                           (2.0f * config_.friction_coefficient * 9.81f);

  // Add reaction time distance: v * t_reaction
  float reaction_distance = ego_velocity_mps_ * config_.reaction_time_s;

  return braking_distance + reaction_distance;
}

std::optional<FCWAlert>
FCWMonitor::check(const std::vector<FusedObject> &objects,
                  uint64_t current_time_ns) {
  FCWAlert most_urgent;
  most_urgent.ttc_s = config_.ttc_threshold_s + 1.0f; // Start above threshold
  bool found_threat = false;

  // Pre-calculate stopping distance for physics-based FCW
  float stopping_distance_m = 0.0f;
  if (config_.use_physics_fcw && ego_velocity_mps_ > 0.5f) {
    stopping_distance_m = calculateStoppingDistance();

    if (g_verbose_mode.load()) {
      std::cout << "[FCW] Ego velocity: " << ego_velocity_mps_
                << " m/s, Stopping distance: " << stopping_distance_m << " m\n";
    }
  }

  for (const auto &obj : objects) {
    // Skip if no radar data
    if (!obj.has_radar)
      continue;

    // Skip if not a relevant class
    if (!isRelevantClass(obj.object_class))
      continue;

    // Skip if range out of bounds
    if (obj.range_m < config_.min_range_m || obj.range_m > config_.max_range_m)
      continue;

    bool ttc_triggered = false;
    bool physics_triggered = false;

    // Check 1: TTC-based alert (original logic)
    if (obj.ttc_s <= config_.ttc_threshold_s) {
      ttc_triggered = true;
    }

    // Check 2: Physics-based alert (can't stop in time)
    if (config_.use_physics_fcw && ego_velocity_mps_ > 0.5f) {
      if (obj.range_m < stopping_distance_m) {
        physics_triggered = true;

        if (g_verbose_mode.load()) {
          std::cout << "[FCW] Physics alert: range=" << obj.range_m
                    << "m < stopping=" << stopping_distance_m << "m\n";
        }
      }
    }

    std::string alert_status = (ttc_triggered || physics_triggered) ? "ALERT TRIGGERED" : "SAFE";
    std::cout << "[StageE: 4_FCW] ID " << obj.object_id 
              << " | Z: " << obj.range_m << "m | V: " << obj.radial_vel_mps 
              << "m/s | TTC: " << obj.ttc_s << "s -> " << alert_status << "\n";

    // Alert if either condition triggers
    if (ttc_triggered || physics_triggered) {
      // Use TTC as urgency metric, but also consider range
      float urgency = obj.ttc_s;
      if (physics_triggered && !ttc_triggered) {
        // Physics-only alert: estimate pseudo-TTC from
        // range/ego_velocity
        urgency = obj.range_m / std::max(ego_velocity_mps_, 0.5f);
      }

      if (urgency < most_urgent.ttc_s) {
        most_urgent.ttc_s = obj.ttc_s;
        most_urgent.range_m = obj.range_m;
        most_urgent.velocity_mps = -obj.radial_vel_mps; // Make positive
        most_urgent.object_class = obj.object_class;
        most_urgent.timestamp_ns = current_time_ns;
        most_urgent.physics_triggered = physics_triggered;
        found_threat = true;
      }
    }
  }

  if (found_threat) {
    if (g_verbose_mode.load()) {
      std::cout << "[FCW] ALERT: TTC=" << most_urgent.ttc_s
                << "s, Range=" << most_urgent.range_m << "m"
                << ", Object=" << most_urgent.object_class
                << (most_urgent.physics_triggered ? " [PHYSICS]" : " [TTC]")
                << "\n";
    }
    return most_urgent;
  }

  return std::nullopt;
}

} // namespace adas
