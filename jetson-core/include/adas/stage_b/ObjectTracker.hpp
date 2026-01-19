// File: include/adas/stage_b/ObjectTracker.hpp
// IoU-based object tracker with persistent IDs (FR35)
#pragma once

#include "adas/common/Types.hpp"

#include <vector>
#include <cstdint>

namespace adas {

/// Tracked object with persistent ID
struct TrackedObject {
    uint32_t id;             // Persistent track ID
    Det detection;           // Latest detection
    uint64_t first_seen_ns;  // When first detected
    uint64_t last_seen_ns;   // When last detected
    int hit_count;           // Consecutive frames with detection
    int miss_count;          // Consecutive frames without detection
    
    TrackedObject()
        : id(0), first_seen_ns(0), last_seen_ns(0), hit_count(0), miss_count(0) {}
};

/// Output batch with tracked objects
struct TrackedBatch {
    Header h;
    std::vector<TrackedObject> tracks;
    uint64_t inference_time_us;
    
    TrackedBatch() : inference_time_us(0) {}
};

/// IoU-based object tracker
/// Matches detections across frames using Intersection over Union
/// Implements FR35: maintain ID for ≥5s on steady lead vehicle
class ObjectTracker {
public:
    struct Config {
        float iou_threshold;        // Minimum IoU to consider a match
        int max_miss_count;         // Frames before track is deleted
        int min_hit_count;          // Hits needed before track is "confirmed"
        
        Config()
            : iou_threshold(0.3f)
            , max_miss_count(30)    // ~1 second at 30 FPS
            , min_hit_count(3) {}
    };
    
    explicit ObjectTracker(const Config& config = Config());
    
    /// Update tracker with new detections
    /// @param detections Batch of detections from detector
    /// @return TrackedBatch with persistent IDs
    TrackedBatch update(const DetBatch& detections);
    
    /// Get current track count
    size_t getTrackCount() const { return tracks_.size(); }
    
private:
    /// Calculate IoU between two bounding boxes
    static float computeIoU(const cv::Rect2f& a, const cv::Rect2f& b);
    
    /// Match detections to existing tracks using Hungarian algorithm (simplified)
    std::vector<std::pair<int, int>> matchDetections(
        const std::vector<Det>& detections);
    
    Config config_;
    std::vector<TrackedObject> tracks_;
    uint32_t next_id_ = 1;  // Start IDs at 1 (0 = invalid)
};

} // namespace adas
