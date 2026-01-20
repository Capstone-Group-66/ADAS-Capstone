// File: src/stage_e/SensorFusion.cpp
// Camera-Radar fusion implementation
// Note: OPS243-A is a 1D ranging radar (no azimuth), so we use a simplified
// fusion approach: match the closest radar target to camera detections
// that are near the center of the frame (boresight assumption).
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
    float center_x = config_.cam_width_px / 2.0f;
    float normalized = (center_x - pixel_x) / config_.cam_width_px;
    return normalized * config_.cam_fov_h_rad;
}

int SensorFusion::findRadarMatch(float cam_azimuth, const RadarTargets& radar) const {
    // For 1D radar (OPS243-A), we ignore azimuth and just check if we have any targets
    // Return the closest target (smallest range) if there are any
    if (radar.targets.empty()) {
        return -1;
    }
    
    // For a 1D forward-facing radar, assume the closest target is the one of interest
    int closest_idx = 0;
    float closest_range = radar.targets[0].range_m;
    
    for (size_t i = 1; i < radar.targets.size(); ++i) {
        if (radar.targets[i].range_m < closest_range) {
            closest_range = radar.targets[i].range_m;
            closest_idx = static_cast<int>(i);
        }
    }
    
    // Only match if camera detection is reasonably centered (within ~30 degrees of center)
    // This is the "boresight cone" where the 1D radar is also pointing
    const float center_tolerance = 30.0f * M_PI / 180.0f;  // 30 degrees
    if (std::abs(cam_azimuth) < center_tolerance) {
        return closest_idx;
    }
    
    return -1;
}

float SensorFusion::computeTTC(float range_m, float radial_vel_mps) const {
    // Negative radial velocity = approaching
    // Always compute TTC if approaching (any negative velocity)
    if (radial_vel_mps < -0.1f) {  // Small threshold to filter noise
        float closing_vel = -radial_vel_mps;  // Make positive
        return range_m / closing_vel;
    }
    return std::numeric_limits<float>::infinity();
}

std::vector<FusedObject> SensorFusion::fuse(const DetBatch& camera, 
                                             const RadarTargets& radar) {
    std::vector<FusedObject> fused;
    
    // Early exit: no fusion possible without both camera AND radar data
    if (camera.dets.empty() || radar.targets.empty()) {
        return fused;  // Return empty vector
    }
    
    fused.reserve(camera.dets.size());
    
    // Always log radar status when we have detections (gated by verbose mode)
    if (g_verbose_mode.load()) {
        std::cout << "[Fusion] Frame: " << camera.dets.size() << " camera dets, " 
                  << radar.targets.size() << " radar targets\n";
        if (!radar.targets.empty()) {
            for (size_t i = 0; i < radar.targets.size() && i < 3; ++i) {
                std::cout << "  Radar[" << i << "]: range=" << radar.targets[i].range_m 
                          << "m, vel=" << radar.targets[i].radial_vel_mps << "m/s\n";
            }
        }
    }
    
    for (const auto& det : camera.dets) {
        FusedObject obj;
        
        // Camera data
        obj.object_class = det.cls;
        obj.score = det.score;
        obj.box_px = det.box_px;
        obj.centroid_px = det.centroid;
        obj.cam_azimuth_rad = pixelToAzimuth(det.centroid.x);
        obj.sources = SRC_CAM_F;
        
        // Try to match with radar (uses 1D radar logic)
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
                std::cout << "[Fusion] Matched " << (det.cls == 0 ? "person" : "object")
                          << " with radar: range=" << target.range_m << "m"
                          << ", vel=" << target.radial_vel_mps << "m/s"
                          << ", TTC=" << (obj.ttc_s < 100 ? std::to_string((int)obj.ttc_s) + "s" : "inf") << "\n";
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
