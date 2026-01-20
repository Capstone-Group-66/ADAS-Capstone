// File: src/stage_b/ObjectDetector.cpp
// YOLOv5 object detection using TensorRT
#include "adas/stage_b/ObjectDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>

namespace adas {

// COCO class names
const std::vector<std::string> ObjectDetector::CLASS_NAMES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

std::string ObjectDetector::getClassName(int class_id) {
    if (class_id >= 0 && class_id < static_cast<int>(CLASS_NAMES.size())) {
        return CLASS_NAMES[class_id];
    }
    return "unknown";
}

#ifdef __aarch64__
// ============================================================================
// TensorRT Implementation (ARM64 / Jetson)
// ============================================================================

void TRTLogger::log(Severity severity, const char* msg) noexcept {
    if (severity <= Severity::kWARNING) {
        std::cout << "[TensorRT] " << msg << std::endl;
    }
}

ObjectDetector::ObjectDetector(const Config& config) : config_(config) {
    logger_ = std::make_unique<TRTLogger>();
    loadEngine();
    if (loaded_) {
        allocateBuffers();
    }
}

ObjectDetector::~ObjectDetector() {
    releaseBuffers();
}

void ObjectDetector::loadEngine() {
    std::cout << "[ObjectDetector] Loading TensorRT engine from " << config_.model_path << "...\n";
    
    // Read engine file
    std::ifstream file(config_.model_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "[ObjectDetector] ERROR: Cannot open engine file: " << config_.model_path << "\n";
        return;
    }
    
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);
    file.close();
    
    // Create runtime and engine
    runtime_.reset(nvinfer1::createInferRuntime(*logger_));
    if (!runtime_) {
        std::cerr << "[ObjectDetector] ERROR: Failed to create TensorRT runtime\n";
        return;
    }
    
    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), size));
    if (!engine_) {
        std::cerr << "[ObjectDetector] ERROR: Failed to deserialize engine\n";
        return;
    }
    
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        std::cerr << "[ObjectDetector] ERROR: Failed to create execution context\n";
        return;
    }
    
    loaded_ = true;
    std::cout << "[ObjectDetector] TensorRT engine loaded successfully\n";
}

void ObjectDetector::allocateBuffers() {
    // Input: [1, 3, 640, 640] FP32
    input_size_ = 1 * 3 * config_.input_width * config_.input_height * sizeof(float);
    
    // YOLOv5 output: [1, 25200, 85] FP32
    // 25200 = 3 anchors * (80*80 + 40*40 + 20*20)
    // 85 = 4 (box) + 1 (obj_conf) + 80 (classes)
    output_elements_ = 25200 * 85;
    output_size_ = output_elements_ * sizeof(float);
    
    // Allocate device memory
    cudaMalloc(&d_input_, input_size_);
    cudaMalloc(&d_output_, output_size_);
    
    // Allocate host output buffer
    h_output_ = new float[output_elements_];
    
    std::cout << "[ObjectDetector] CUDA buffers allocated (input: " 
              << input_size_ / 1024 << " KB, output: " << output_size_ / 1024 << " KB)\n";
}

void ObjectDetector::releaseBuffers() {
    if (d_input_) cudaFree(d_input_);
    if (d_output_) cudaFree(d_output_);
    if (h_output_) delete[] h_output_;
    d_input_ = nullptr;
    d_output_ = nullptr;
    h_output_ = nullptr;
}

cv::Mat ObjectDetector::preprocess(const cv::Mat& frame) {
    cv::Mat resized, blob;
    
    // Resize to model input size
    cv::resize(frame, resized, cv::Size(config_.input_width, config_.input_height));
    
    // Convert BGR to RGB and normalize to [0,1]
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    resized.convertTo(blob, CV_32FC3, 1.0f / 255.0f);
    
    return blob;
}

DetBatch ObjectDetector::detect(const cv::Mat& frame, const Header& header) {
    DetBatch result;
    result.h = header;
    result.inference_time_us = 0;
    
    if (!loaded_ || frame.empty()) {
        return result;
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Preprocess
    cv::Mat blob = preprocess(frame);
    
    // Convert HWC to CHW format for TensorRT
    std::vector<cv::Mat> channels(3);
    cv::split(blob, channels);
    
    // Copy to device (CHW layout)
    float* input_data = new float[3 * config_.input_width * config_.input_height];
    int channel_size = config_.input_width * config_.input_height;
    for (int c = 0; c < 3; ++c) {
        memcpy(input_data + c * channel_size, channels[c].data, channel_size * sizeof(float));
    }
    
    cudaMemcpy(d_input_, input_data, input_size_, cudaMemcpyHostToDevice);
    delete[] input_data;
    
    // Run inference
    void* bindings[] = {d_input_, d_output_};
    context_->executeV2(bindings);
    
    // Copy output back to host
    cudaMemcpy(h_output_, d_output_, output_size_, cudaMemcpyDeviceToHost);
    
    // Postprocess
    std::vector<Detection> internal_dets = postprocess(h_output_, frame.size());
    
    // Convert to Det struct with centroid
    for (const auto& det : internal_dets) {
        cv::Rect2f box_f(static_cast<float>(det.box.x), static_cast<float>(det.box.y),
                         static_cast<float>(det.box.width), static_cast<float>(det.box.height));
        result.dets.emplace_back(box_f, det.class_id, det.confidence);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    result.inference_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    
    return result;
}

std::vector<Detection> ObjectDetector::postprocess(float* output, const cv::Size& original_size) {
    std::vector<Detection> detections;
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    // YOLOv5 output: [25200, 85]
    // Row format: cx, cy, w, h, obj_conf, class_scores[80]
    const int num_detections = 25200;
    const int num_outputs = 85;
    
    float x_scale = static_cast<float>(original_size.width) / config_.input_width;
    float y_scale = static_cast<float>(original_size.height) / config_.input_height;
    
    for (int i = 0; i < num_detections; ++i) {
        float* row = output + i * num_outputs;
        
        float obj_conf = row[4];
        if (obj_conf < config_.confidence_threshold) continue;
        
        // Find best class
        float max_class_score = 0;
        int max_class_id = 0;
        for (int j = 5; j < num_outputs; ++j) {
            if (row[j] > max_class_score) {
                max_class_score = row[j];
                max_class_id = j - 5;
            }
        }
        
        float confidence = obj_conf * max_class_score;
        if (confidence < config_.confidence_threshold) continue;
        
        // Extract box
        float cx = row[0];
        float cy = row[1];
        float w = row[2];
        float h = row[3];
        
        int x = static_cast<int>((cx - w / 2) * x_scale);
        int y = static_cast<int>((cy - h / 2) * y_scale);
        int width = static_cast<int>(w * x_scale);
        int height = static_cast<int>(h * y_scale);
        
        boxes.push_back(cv::Rect(x, y, width, height));
        confidences.push_back(confidence);
        class_ids.push_back(max_class_id);
    }
    
    // Apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, config_.confidence_threshold, 
                      config_.nms_threshold, indices);
    
    for (int idx : indices) {
        Detection det;
        det.class_id = class_ids[idx];
        det.confidence = confidences[idx];
        det.box = boxes[idx];
        det.class_name = getClassName(det.class_id);
        detections.push_back(det);
    }
    
    return detections;
}

#else
// ============================================================================
// Stub Implementation (non-ARM platforms for cross-compilation)
// ============================================================================

ObjectDetector::ObjectDetector(const Config& config) : config_(config) {
    std::cout << "[ObjectDetector] STUB: TensorRT not available (non-ARM platform)\n";
    loaded_ = false;
}

ObjectDetector::~ObjectDetector() {}

DetBatch ObjectDetector::detect(const cv::Mat& frame, const Header& header) {
    DetBatch result;
    result.h = header;
    result.inference_time_us = 0;
    // Return empty detections on non-ARM platforms
    return result;
}

#endif // __aarch64__

} // namespace adas
