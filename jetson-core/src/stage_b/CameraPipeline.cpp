// File: src/stage_b/CameraPipeline.cpp
// Stage B Camera Pipeline implementation
#include "adas/stage_b/CameraPipeline.hpp"

#include <opencv2/imgproc.hpp>

#include <iostream>

namespace adas {

// ============================================================================
//                            CameraPipeline
// ============================================================================

CameraPipeline::CameraPipeline(Mount mount,
                               SPSCQueue<CameraFrameData, 8>& input_queue,
                               SPSCQueue<DetBatch, 8>& output_queue,
                               const std::string& calib_dir,
                               const std::string& model_path)
    : mount_(mount),
      input_queue_(input_queue),
      output_queue_(output_queue) {
    
    // Create preprocessor (handles missing calibration gracefully)
    preprocessor_ = std::make_unique<CameraPreprocessor>(mount, calib_dir);
    
    // Create detector (handles missing model gracefully)
    ObjectDetector::Config det_config;
    det_config.model_path = model_path;
    detector_ = std::make_unique<ObjectDetector>(det_config);
}

CameraPipeline::~CameraPipeline() {
    stop();
}

void CameraPipeline::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }
    
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&CameraPipeline::threadFunc, this);
    
    std::cout << "[CameraPipeline] Started for " << mountToString(mount_) << "\n";
}

void CameraPipeline::stop() {
    if (!running_.load(std::memory_order_relaxed)) {
        return;
    }
    
    running_.store(false, std::memory_order_relaxed);
    
    if (thread_.joinable()) {
        thread_.join();
    }
    
    std::cout << "[CameraPipeline] Stopped for " << mountToString(mount_) 
              << " (processed " << frames_processed_.load() << " frames)\n";
}

void CameraPipeline::threadFunc() {
    std::cout << "[CameraPipeline] Thread started for " << mountToString(mount_) << "\n";
    
    while (running_.load(std::memory_order_relaxed)) {
        CameraFrameData frame;
        
        // Try to pop a frame from input queue
        if (!input_queue_.try_pop(frame)) {
            // No frame available, sleep briefly
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        healthy_.store(true, std::memory_order_relaxed);
        
        // Convert frame data to cv::Mat
        if (frame.data.empty() || frame.width <= 0 || frame.height <= 0) {
            continue;
        }
        
        cv::Mat image(frame.height, frame.width, CV_8UC3, frame.data.data());
        
        // Step 1: Undistortion
        cv::Mat undistorted = preprocessor_->process(image, false);
        
        // Step 2: Object detection
        DetBatch detections = detector_->detect(undistorted, frame.h);
        
        // Update statistics
        frames_processed_.fetch_add(1, std::memory_order_relaxed);
        total_inference_time_us_.fetch_add(detections.inference_time_us, 
                                           std::memory_order_relaxed);
        
        // Push to output queue (may drop if full - that's OK per "freshness over completeness")
        output_queue_.try_push(detections);
    }
    
    healthy_.store(false, std::memory_order_relaxed);
}

uint64_t CameraPipeline::getAvgInferenceTimeUs() const {
    uint64_t frames = frames_processed_.load(std::memory_order_relaxed);
    if (frames == 0) return 0;
    return total_inference_time_us_.load(std::memory_order_relaxed) / frames;
}

// ============================================================================
//                            StageBManager
// ============================================================================

StageBManager::StageBManager(const std::string& calib_dir, 
                             const std::string& model_path)
    : calib_dir_(calib_dir), model_path_(model_path) {}

StageBManager::~StageBManager() {
    stop();
}

void StageBManager::addCamera(Mount mount,
                              SPSCQueue<CameraFrameData, 8>& input_queue,
                              SPSCQueue<DetBatch, 8>& output_queue) {
    pipelines_.push_back(std::make_unique<CameraPipeline>(
        mount, input_queue, output_queue, calib_dir_, model_path_));
    
    std::cout << "[StageBManager] Added camera pipeline for " 
              << mountToString(mount) << "\n";
}

void StageBManager::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }
    
    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "          STAGE B: CAMERA PREPROC + INFERENCE                 \n";
    std::cout << "==============================================================\n";
    std::cout << "  Starting " << pipelines_.size() << " camera pipeline(s)...\n";
    std::cout << "==============================================================\n";
    std::cout << "\n";
    
    for (auto& pipeline : pipelines_) {
        pipeline->start();
    }
    
    running_.store(true, std::memory_order_relaxed);
}

void StageBManager::stop() {
    if (!running_.load(std::memory_order_relaxed)) {
        return;
    }
    
    std::cout << "[StageBManager] Stopping all camera pipelines...\n";
    
    for (auto& pipeline : pipelines_) {
        pipeline->stop();
    }
    
    running_.store(false, std::memory_order_relaxed);
    std::cout << "[StageBManager] All pipelines stopped\n";
}

bool StageBManager::isRunning() const {
    return running_.load(std::memory_order_relaxed);
}

void StageBManager::printStatus() const {
    std::cout << "\n";
    std::cout << "----------------------------------------------------------------\n";
    std::cout << "                    STAGE B STATUS                              \n";
    std::cout << "----------------------------------------------------------------\n";
    
    for (const auto& pipeline : pipelines_) {
        std::cout << "  " << mountToString(pipeline->getMount()) << ": "
                  << (pipeline->isHealthy() ? "[OK] " : "[--] ")
                  << pipeline->getFramesProcessed() << " frames, "
                  << "avg " << pipeline->getAvgInferenceTimeUs() / 1000.0 << "ms\n";
    }
    
    std::cout << "----------------------------------------------------------------\n";
}

} // namespace adas
