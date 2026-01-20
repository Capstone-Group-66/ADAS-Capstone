// File: include/adas/stage_b/ObjectDetector.hpp
// YOLOv5 object detection for Stage B using TensorRT
#pragma once

#include "adas/common/Types.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

// TensorRT headers (available on Jetson via JetPack)
#ifdef __aarch64__
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace adas {

// Note: Det and DetBatch are defined in Types.hpp

/// Internal detection result (before conversion to Det)
struct Detection {
    int class_id;           // Class index (0=person, 2=car, etc.)
    float confidence;       // Detection confidence [0,1]
    cv::Rect box;           // Bounding box in image coordinates
    std::string class_name; // Human-readable class name
};

/// TensorRT Logger (required by TensorRT API)
#ifdef __aarch64__
class TRTLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override;
};
#endif

/// ObjectDetector: YOLOv5 Nano inference using TensorRT
/// Falls back to stub on non-ARM platforms (for cross-compilation)
class ObjectDetector {
  public:
    /// Configuration for detector
    struct Config {
        std::string model_path;      // Path to .engine file (TensorRT)
        float confidence_threshold;
        float nms_threshold;
        int input_width;
        int input_height;
        int num_classes;
        
        Config() 
            : model_path("models/yolov5n.engine")
            , confidence_threshold(0.40f)  // Higher threshold = less flicker
            , nms_threshold(0.45f)
            , input_width(640)
            , input_height(640)
            , num_classes(80) {}
    };

    /// Constructor - loads TensorRT engine
    explicit ObjectDetector(const Config& config = Config());
    
    /// Destructor - releases CUDA resources
    ~ObjectDetector();

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
    void loadEngine();
    void allocateBuffers();
    void releaseBuffers();
    
    cv::Mat preprocess(const cv::Mat& frame);
    std::vector<Detection> postprocess(float* output, const cv::Size& original_size);

    Config config_;
    bool loaded_ = false;
    
#ifdef __aarch64__
    // TensorRT runtime objects
    std::unique_ptr<TRTLogger> logger_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    
    // CUDA buffers
    void* d_input_ = nullptr;   // Device input buffer
    void* d_output_ = nullptr;  // Device output buffer
    float* h_output_ = nullptr; // Host output buffer
    
    // Buffer sizes
    size_t input_size_ = 0;
    size_t output_size_ = 0;
    int output_elements_ = 0;
#endif
    
    // COCO class names (first 80)
    static const std::vector<std::string> CLASS_NAMES;
};

} // namespace adas
