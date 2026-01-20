// File: src/stage_b/ObjectDetector.cpp
// YOLOv8 object detection implementation
#include "adas/stage_b/ObjectDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>

namespace adas {

// COCO class names (80 classes)
const std::vector<std::string> ObjectDetector::CLASS_NAMES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
};

ObjectDetector::ObjectDetector(const Config& config) : config_(config) {
    loadModel();
}

void ObjectDetector::loadModel() {
    if (!std::filesystem::exists(config_.model_path)) {
        std::cerr << "[ObjectDetector] WARNING: Model not found at " 
                  << config_.model_path << "\n";
        std::cerr << "[ObjectDetector] Object detection will be disabled\n";
        return;
    }
    
    try {
        std::cout << "[ObjectDetector] Loading model from " << config_.model_path << "...\n";
        net_ = cv::dnn::readNetFromONNX(config_.model_path);
        
        // Use CPU backend by default for maximum compatibility
        // CUDA backend can cause assertion failures if not properly configured
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "[ObjectDetector] Using CPU backend\n";
        
        loaded_ = true;
        std::cout << "[ObjectDetector] Model loaded successfully\n";
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ObjectDetector] ERROR loading model: " << e.what() << "\n";
        loaded_ = false;
    }
}

cv::Mat ObjectDetector::preprocess(const cv::Mat& frame) {
    cv::Mat blob;
    cv::dnn::blobFromImage(
        frame, 
        blob, 
        1.0 / 255.0,  // Scale to [0,1]
        cv::Size(config_.input_width, config_.input_height),
        cv::Scalar(0, 0, 0),  // No mean subtraction for YOLO
        true,   // swapRB (BGR -> RGB)
        false   // crop
    );
    return blob;
}

std::vector<Detection> ObjectDetector::postprocess(const cv::Mat& output, 
                                                    const cv::Size& original_size) {
    std::vector<Detection> detections;
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    // YOLOv5 output shape: [1, 25200, 85] (cx, cy, w, h, obj_conf, 80 classes)
    // No transpose needed typically, it comes as rows
    // shape[1] is 85, shape[2] is 25200 (or vice versa depending on export)
    // OpenCV typically returns [1, 25200, 85] in modern parsers, but sometimes [1, 85, 25200]
    
    cv::Mat data = output;
    
    // Check if we need to reshape/transpose
    // We expect final shape to be [N, 85] where N is number of anchors
    if (data.dims > 2) {
        // Flatten batch dim: [1, 25200, 85] -> [25200, 85]
        data = data.reshape(1, data.size[1]);
    }
    
    // If we have [85, 25200], transpose to [25200, 85]
    if (data.cols > data.rows && data.cols == 25200) {
        cv::transpose(data, data);
    }
    
    float x_scale = static_cast<float>(original_size.width) / config_.input_width;
    float y_scale = static_cast<float>(original_size.height) / config_.input_height;
    
    for (int i = 0; i < data.rows; ++i) {
        // Column 4 is objectness confidence
        float obj_conf = data.at<float>(i, 4);
        
        if (obj_conf < config_.confidence_threshold) continue;

        // Columns 5..84 are class scores
        float max_class_score = 0;
        int max_class_id = 0;
        
        // Find best class
        for (int j = 5; j < data.cols; ++j) {
            float score = data.at<float>(i, j);
            if (score > max_class_score) {
                max_class_score = score;
                max_class_id = j - 5;
            }
        }
        
        // Final confidence = obj_conf * class_score
        float confidence = obj_conf * max_class_score;
        
        if (confidence >= config_.confidence_threshold) {
             // First 4 values: cx, cy, w, h
            float cx = data.at<float>(i, 0);
            float cy = data.at<float>(i, 1);
            float w = data.at<float>(i, 2);
            float h = data.at<float>(i, 3);
            
            // Convert from center format to corner format and scale
            int x = static_cast<int>((cx - w / 2) * x_scale);
            int y = static_cast<int>((cy - h / 2) * y_scale);
            int width = static_cast<int>(w * x_scale);
            int height = static_cast<int>(h * y_scale);
            
            boxes.push_back(cv::Rect(x, y, width, height));
            confidences.push_back(confidence);
            class_ids.push_back(max_class_id);
        }
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

DetBatch ObjectDetector::detect(const cv::Mat& frame, const Header& header) {
    DetBatch result;
    result.h = header;
    result.inference_time_us = 0;
    
    if (!loaded_ || frame.empty()) {
        return result;  // Return empty batch if model not loaded
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Preprocess
    cv::Mat blob = preprocess(frame);
    net_.setInput(blob);
    
    // Inference
    cv::Mat output = net_.forward();
    
    // Postprocess - get internal detections
    std::vector<Detection> internal_dets = postprocess(output, frame.size());
    
    // Convert to Det struct with centroid
    for (const auto& det : internal_dets) {
        cv::Rect2f box_f(static_cast<float>(det.box.x), static_cast<float>(det.box.y),
                         static_cast<float>(det.box.width), static_cast<float>(det.box.height));
        result.dets.emplace_back(box_f, det.class_id, det.confidence);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    result.inference_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end - start).count();
    
    return result;
}

std::string ObjectDetector::getClassName(int class_id) {
    if (class_id >= 0 && class_id < static_cast<int>(CLASS_NAMES.size())) {
        return CLASS_NAMES[class_id];
    }
    return "unknown";
}

} // namespace adas
