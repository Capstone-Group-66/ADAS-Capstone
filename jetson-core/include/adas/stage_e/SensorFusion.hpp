// File: include/adas/stage_e/SensorFusion.hpp
// Camera-Radar fusion for FCW vertical slice
#pragma once

#include "adas/common/Types.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <optional>
#include <vector>

namespace adas {

/// Fused object combining camera detection with radar measurement
struct FusedObject {
    uint32_t track_id;          // From camera tracker (if available)
    int object_class;           // From camera detection
    float score;                // Detection confidence
    
    // Camera data
    cv::Rect2f box_px;          // Bounding box in pixels
    cv::Point2f centroid_px;    // Center in pixels
    float cam_azimuth_rad;      // Computed angular bearing
    
    // Radar data (if matched)
    bool has_radar;             // True if radar target matched
    float range_m;              // From radar
    float radial_vel_mps;       // From radar (negative = approaching)
    float radar_azimuth_rad;    // From radar target
    
    // Computed
    float ttc_s;                // Time-to-collision (positive, or INFINITY if not approaching)
    uint16_t sources;           // SensorSource bitmask
    
    FusedObject()
        : track_id(0), object_class(0), score(0),
          box_px(), centroid_px(), cam_azimuth_rad(0),
          has_radar(false), range_m(0), radial_vel_mps(0), radar_azimuth_rad(0),
          ttc_s(std::numeric_limits<float>::infinity()),
          sources(SRC_NONE) {}
};

/// Configuration for sensor fusion
struct FusionConfig {
    // Camera parameters (FrontCam)
    float cam_fov_h_rad = 60.0f * M_PI / 180.0f;  // Horizontal FOV in radians
    float cam_width_px = 1280.0f;                  // Frame width
    
    // Association parameters
    float azimuth_match_threshold_rad = 5.0f * M_PI / 180.0f;  // 5 degrees
    
    // TTC parameters
    float min_closing_vel_mps = 0.5f;  // Minimum velocity to compute TTC
    
    FusionConfig() = default;
};

/// SensorFusion: Fuses camera detections with radar targets
/// Uses boresight assumption (camera and radar co-aligned forward)
class SensorFusion {
public:
    explicit SensorFusion(const FusionConfig& config = FusionConfig());
    
    /// Fuse camera detections with radar targets
    /// @param camera Camera detections from Stage B
    /// @param radar Radar targets from Stage A
    /// @return Vector of fused objects with TTC
    std::vector<FusedObject> fuse(const DetBatch& camera, const RadarTargets& radar);
    
private:
    /// Convert pixel X coordinate to azimuth angle
    /// @param pixel_x Horizontal pixel position
    /// @return Azimuth in radians (center=0, left=+, right=-)
    float pixelToAzimuth(float pixel_x) const;
    
    /// Find best matching radar target for a camera detection
    /// @param cam_azimuth Camera detection azimuth
    /// @param radar All radar targets
    /// @return Index into radar.targets, or -1 if no match
    int findRadarMatch(float cam_azimuth, const RadarTargets& radar) const;
    
    /// Compute TTC from range and velocity
    /// @param range_m Distance to target
    /// @param radial_vel_mps Radial velocity (negative = approaching)
    /// @return TTC in seconds, or INFINITY if not approaching
    float computeTTC(float range_m, float radial_vel_mps) const;
    
    FusionConfig config_;
};

} // namespace adas
