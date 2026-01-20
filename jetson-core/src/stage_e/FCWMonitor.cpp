// File: src/stage_e/FCWMonitor.cpp
// Forward Collision Warning implementation
#include "adas/stage_e/FCWMonitor.hpp"
#include "adas/common/Globals.hpp"

#include <iostream>

namespace adas {

FCWMonitor::FCWMonitor(const Config& config) : config_(config) {}

bool FCWMonitor::isRelevantClass(int cls) {
    // COCO classes relevant for FCW:
    // 0 = person, 1 = bicycle, 2 = car, 3 = motorcycle, 
    // 5 = bus, 7 = truck
    switch (cls) {
        case 0:  // person
        case 1:  // bicycle
        case 2:  // car
        case 3:  // motorcycle
        case 5:  // bus
        case 7:  // truck
            return true;
        default:
            return false;
    }
}

std::optional<FCWAlert> FCWMonitor::check(const std::vector<FusedObject>& objects,
                                          uint64_t current_time_ns) {
    FCWAlert most_urgent;
    most_urgent.ttc_s = config_.ttc_threshold_s + 1.0f;  // Start above threshold
    bool found_threat = false;
    
    for (const auto& obj : objects) {
        // Skip if no radar data
        if (!obj.has_radar) continue;
        
        // Skip if not a relevant class
        if (!isRelevantClass(obj.object_class)) continue;
        
        // Skip if range out of bounds
        if (obj.range_m < config_.min_range_m || obj.range_m > config_.max_range_m) continue;
        
        // Skip if not approaching (infinite TTC)
        if (obj.ttc_s > config_.ttc_threshold_s) continue;
        
        // Check if this is the most urgent threat
        if (obj.ttc_s < most_urgent.ttc_s) {
            most_urgent.ttc_s = obj.ttc_s;
            most_urgent.range_m = obj.range_m;
            most_urgent.velocity_mps = -obj.radial_vel_mps;  // Make positive
            most_urgent.object_class = obj.object_class;
            most_urgent.timestamp_ns = current_time_ns;
            found_threat = true;
        }
    }
    
    if (found_threat) {
        if (g_verbose_mode.load()) {
            std::cout << "[FCW] ALERT: TTC=" << most_urgent.ttc_s 
                      << "s < " << config_.ttc_threshold_s << "s threshold!"
                      << " Object=" << most_urgent.object_class 
                      << ", Range=" << most_urgent.range_m << "m\n";
        }
        return most_urgent;
    }
    
    return std::nullopt;
}

} // namespace adas
