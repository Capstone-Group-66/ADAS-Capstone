// File: src/stage_b/ObjectTracker.cpp
// IoU-based object tracker implementation
#include "adas/stage_b/ObjectTracker.hpp"

#include <algorithm>
#include <limits>

namespace adas {

ObjectTracker::ObjectTracker(const Config &config) : config_(config) {}

float ObjectTracker::computeIoU(const cv::Rect2f &a, const cv::Rect2f &b) {
    // Compute intersection
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);

    float intersection_width = std::max(0.0f, x2 - x1);
    float intersection_height = std::max(0.0f, y2 - y1);
    float intersection_area = intersection_width * intersection_height;

    // Compute union
    float area_a = a.width * a.height;
    float area_b = b.width * b.height;
    float union_area = area_a + area_b - intersection_area;

    if (union_area <= 0)
        return 0.0f;

    return intersection_area / union_area;
}

std::vector<std::pair<int, int>>
ObjectTracker::matchDetections(const std::vector<Det> &detections) {

    std::vector<std::pair<int, int>> matches;
    std::vector<bool> det_matched(detections.size(), false);
    std::vector<bool> track_matched(tracks_.size(), false);

    // Simple greedy matching (could be replaced with Hungarian algorithm)
    // For each detection, find best matching track
    for (size_t d = 0; d < detections.size(); ++d) {
        float best_iou = config_.iou_threshold;
        int best_track = -1;

        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (track_matched[t])
                continue;

            // Only match same class
            if (detections[d].cls != tracks_[t].detection.cls)
                continue;

            float iou = computeIoU(detections[d].box_px, tracks_[t].detection.box_px);
            if (iou > best_iou) {
                best_iou = iou;
                best_track = static_cast<int>(t);
            }
        }

        if (best_track >= 0) {
            matches.emplace_back(static_cast<int>(d), best_track);
            det_matched[d] = true;
            track_matched[best_track] = true;
        }
    }

    return matches;
}

TrackedBatch ObjectTracker::update(const DetBatch &detections) {
    TrackedBatch result;
    result.h = detections.h;
    result.inference_time_us = detections.inference_time_us;

    uint64_t current_time = detections.h.t_ingest_ns;

    // Match detections to existing tracks
    auto matches = matchDetections(detections.dets);

    // Track which detections and tracks are matched
    std::vector<bool> det_matched(detections.dets.size(), false);
    std::vector<bool> track_updated(tracks_.size(), false);

    // Update matched tracks
    for (const auto &match : matches) {
        int det_idx = match.first;
        int track_idx = match.second;

        tracks_[track_idx].detection = detections.dets[det_idx];
        tracks_[track_idx].last_seen_ns = current_time;
        tracks_[track_idx].hit_count++;
        tracks_[track_idx].miss_count = 0;

        det_matched[det_idx] = true;
        track_updated[track_idx] = true;
    }

    // Increment miss count for unmatched tracks
    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!track_updated[t]) {
            tracks_[t].miss_count++;
        }
    }

    // Create new tracks for unmatched detections
    for (size_t d = 0; d < detections.dets.size(); ++d) {
        if (!det_matched[d]) {
            TrackedObject new_track;
            new_track.id = next_id_++;
            new_track.detection = detections.dets[d];
            new_track.first_seen_ns = current_time;
            new_track.last_seen_ns = current_time;
            new_track.hit_count = 1;
            new_track.miss_count = 0;
            tracks_.push_back(new_track);
        }
    }

    // Remove stale tracks (exceeds max_miss_count)
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const TrackedObject &t) {
                                     return t.miss_count > config_.max_miss_count;
                                 }),
                  tracks_.end());

    // Output only confirmed tracks (hit_count >= min_hit_count)
    for (const auto &track : tracks_) {
        if (track.hit_count >= config_.min_hit_count) {
            result.tracks.push_back(track);
        }
    }

    return result;
}

} // namespace adas
