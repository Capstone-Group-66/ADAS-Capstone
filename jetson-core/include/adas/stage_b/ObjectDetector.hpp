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

// Note: Det and DetBatch are defined in Types.hpp

/// Internal detection result (before conversion to Det)
struct Detection {
    int class_id;           // Class index (0=person, 2=car, etc.)
    float confidence;       // Detection confidence [0,1]
    cv::Rect box;           // Bounding box in image coordinates
    std::string class_name; // Human-readable class name
};

/// ObjectDetector: YOLOv8 Nano inference for object detection
/// Uses OpenCV DNN (fallback) or TensorRT (if available)
class ObjectDetector {
  public:
    /// Configuration for detector
    struct Config {
        std::string model_path;
        float confidence_threshold;
        float nms_threshold;
        int input_width;
        int input_height;
        bool use_cuda;
        
        Config() 
            : model_path("models/yolov8n.onnx")
            , confidence_threshold(0.25f)
            , nms_threshold(0.45f)
            , input_width(640)
            , input_height(640)
            , use_cuda(true) {}
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
