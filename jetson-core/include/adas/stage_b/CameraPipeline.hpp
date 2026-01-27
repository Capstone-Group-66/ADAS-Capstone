// File: include/adas/stage_b/CameraPipeline.hpp
// Stage B Camera Pipeline - combines preprocessing and inference
#pragma once

#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"
#include "adas/stage_b/CameraPreprocessor.hpp"
#include "adas/stage_b/ObjectDetector.hpp"

#include <atomic>
#include <memory>
#include <thread>

namespace adas {

/// CameraPipeline: Consumer-Producer for a single camera mount
/// Consumes: CameraFrameData from Stage A
/// Produces: DetBatch for Stage E
class CameraPipeline {
  public:
    /// Constructor
    /// @param mount Camera mount this pipeline processes
    /// @param input_queue Queue from Stage A (CameraIngest)
    /// @param output_queue Queue to Stage E (Fusion)
    /// @param calib_dir Directory containing calibration files
    /// @param model_path Path to ONNX model file
    CameraPipeline(Mount mount, SPSCQueue<CameraFrameData, 8> &input_queue,
                   SPSCQueue<DetBatch, 8> &output_queue,
                   const std::string &calib_dir = "config/calibration",
                   const std::string &model_path = "models/yolov8n.onnx");

    ~CameraPipeline();

    // Non-copyable
    CameraPipeline(const CameraPipeline &) = delete;
    CameraPipeline &operator=(const CameraPipeline &) = delete;

    /// Start the processing thread
    void start();

    /// Stop the processing thread gracefully
    void stop();

    /// Check if pipeline is running
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    /// Check if pipeline is healthy (processing frames)
    bool isHealthy() const { return healthy_.load(std::memory_order_relaxed); }

    /// Get the mount this pipeline processes
    Mount getMount() const { return mount_; }

    /// Get statistics
    uint64_t getFramesProcessed() const { return frames_processed_.load(); }
    uint64_t getAvgInferenceTimeUs() const;

  private:
    void threadFunc();

    Mount mount_;
    SPSCQueue<CameraFrameData, 8> &input_queue_;
    SPSCQueue<DetBatch, 8> &output_queue_;

    std::unique_ptr<CameraPreprocessor> preprocessor_;
    std::unique_ptr<ObjectDetector> detector_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> healthy_{false};
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> total_inference_time_us_{0};
};

/// StageBManager: Manages all camera pipelines for Stage B
class StageBManager {
  public:
    /// Constructor
    /// @param calib_dir Directory containing calibration files
    /// @param model_path Path to ONNX model file
    explicit StageBManager(const std::string &calib_dir = "config/calibration",
                           const std::string &model_path = "models/yolov8n.onnx");

    ~StageBManager();

    /// Add a camera pipeline for a mount
    /// @param mount Camera mount
    /// @param input_queue Queue from Stage A
    /// @param output_queue Queue to Stage E
    void addCamera(Mount mount, SPSCQueue<CameraFrameData, 8> &input_queue,
                   SPSCQueue<DetBatch, 8> &output_queue);

    /// Start all pipelines
    void start();

    /// Stop all pipelines gracefully
    void stop();

    /// Check if all pipelines are running
    bool isRunning() const;

    /// Print status to stdout
    void printStatus() const;

  private:
    std::string calib_dir_;
    std::string model_path_;
    std::vector<std::unique_ptr<CameraPipeline>> pipelines_;
    std::atomic<bool> running_{false};
};

} // namespace adas
