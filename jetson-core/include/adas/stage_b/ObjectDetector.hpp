// File: include/adas/stage_b/ObjectDetector.hpp
// YOLOv8 object detection for Stage B
#pragma once

#include "adas/common/Types.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <memory>
#include <string>
#include <vector>

namespace adas {

/// Single detection result
struct Detection {
    int class_id;           // Class index (0=person, 2=car, etc.)
    float confidence;       // Detection confidence [0,1]
    cv::Rect box;           // Bounding box in image coordinates
    std::string class_name; // Human-readable class name
};

/// Batch of detections from a single frame
struct DetBatch {
    Header h;                       // Inherited from source frame
    std::vector<Detection> dets;    // All detections in frame
    uint64_t inference_time_us;     // Inference latency in microseconds
};

/// ObjectDetector: YOLOv8 Nano inference for object detection
/// Uses OpenCV DNN (fallback) or TensorRT (if available)
class ObjectDetector {
  public:
    /// Configuration for detector
    struct Config {
        std::string model_path = "models/yolov8n.onnx";
        float confidence_threshold = 0.25f;
        float nms_threshold = 0.45f;
        int input_width = 640;
        int input_height = 640;
        bool use_cuda = true;  // Try CUDA backend first
    };

    /// Constructor - loads model
    explicit ObjectDetector(const Config& config = Config());

    /// Check if model was loaded successfully
    bool isLoaded() const { return loaded_; }

    /// Run inference on a frame
    /// @param frame BGR image (any size, will be resized)
    /// @param header Header from source frame
    /// @return DetBatch with all detections
    DetBatch detect(const cv::Mat& frame, const Header& header);

    /// Get class name by ID
    static std::string getClassName(int class_id);

  private:
    void loadModel();
    cv::Mat preprocess(const cv::Mat& frame);
    std::vector<Detection> postprocess(const cv::Mat& output, const cv::Size& original_size);

    Config config_;
    cv::dnn::Net net_;
    bool loaded_ = false;
    
    // COCO class names (first 80)
    static const std::vector<std::string> CLASS_NAMES;
};

} // namespace adas
