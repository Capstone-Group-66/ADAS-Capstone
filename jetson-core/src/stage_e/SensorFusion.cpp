// File: src/stage_e/SensorFusion.cpp
// Camera-Radar fusion implementation
#include "adas/stage_e/SensorFusion.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "adas/common/Globals.hpp"

namespace adas {

SensorFusion::SensorFusion(const FusionConfig& config) : config_(config) {}

float SensorFusion::pixelToAzimuth(float pixel_x) const {
    // Boresight assumption: camera center = 0 azimuth
    // Left of center = positive azimuth, right = negative
    // This matches radar convention
    float center_x = config_.cam_width_px / 2.0f;
    float normalized = (center_x - pixel_x) / config_.cam_width_px;
    return normalized * config_.cam_fov_h_rad;
}

int SensorFusion::findRadarMatch(float cam_azimuth, const RadarTargets& radar) const {
    int best_idx = -1;
    float best_diff = config_.azimuth_match_threshold_rad;
    
    for (size_t i = 0; i < radar.targets.size(); ++i) {
        float diff = std::abs(radar.targets[i].azimuth_rad - cam_azimuth);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx = static_cast<int>(i);
        }
    }
    
    return best_idx;
}

float SensorFusion::computeTTC(float range_m, float radial_vel_mps) const {
    // Negative radial velocity = approaching
    // Only compute TTC if approaching faster than threshold
    if (radial_vel_mps < -config_.min_closing_vel_mps) {
        float closing_vel = -radial_vel_mps;  // Make positive
        return range_m / closing_vel;
    }
    return std::numeric_limits<float>::infinity();
}

std::vector<FusedObject> SensorFusion::fuse(const DetBatch& camera, 
                                             const RadarTargets& radar) {
    std::vector<FusedObject> fused;
    fused.reserve(camera.dets.size());
    
    for (const auto& det : camera.dets) {
        FusedObject obj;
        
        // Camera data
        obj.object_class = det.cls;
        obj.score = det.score;
        obj.box_px = det.box_px;
        obj.centroid_px = det.centroid;
        obj.cam_azimuth_rad = pixelToAzimuth(det.centroid.x);
        obj.sources = SRC_CAM_F;
        
        // Try to match with radar
        int radar_idx = findRadarMatch(obj.cam_azimuth_rad, radar);
        
        if (radar_idx >= 0) {
            const RadarTarget& target = radar.targets[radar_idx];
            
            obj.has_radar = true;
            obj.range_m = target.range_m;
            obj.radial_vel_mps = target.radial_vel_mps;
            obj.radar_azimuth_rad = target.azimuth_rad;
            obj.sources |= SRC_RAD_F;
            
            // Compute TTC
            obj.ttc_s = computeTTC(target.range_m, target.radial_vel_mps);
            
            if (g_verbose_mode.load()) {
                std::cout << "[Fusion] Matched cam_det (cls=" << det.cls 
                          << ") with radar: range=" << target.range_m << "m"
                          << ", vel=" << target.radial_vel_mps << "m/s"
                          << ", TTC=" << obj.ttc_s << "s\n";
            }
        } else {
            // Camera only - no range/velocity data
            obj.has_radar = false;
            obj.range_m = 0;
            obj.radial_vel_mps = 0;
            obj.ttc_s = std::numeric_limits<float>::infinity();
        }
        
        fused.push_back(obj);
    }
    
    return fused;
}

} // namespace adas
