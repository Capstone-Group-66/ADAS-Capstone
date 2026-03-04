// File: include/adas/stage_b/FrontCamDeepStream.hpp
// Stage B — DeepStream 6.0 Front Camera Pipeline
//
// Replaces the previous YOLOv5/TensorRT ObjectDetector + CameraPipeline pair
// for the FrontCam mount. This module owns the entire GStreamer pipeline:
//
//   V4L2src (YUYV, 1280x720 @ 10 FPS)
//   └─► nvv4l2decoder (CUDA decode)
//       └─► nvstreammux
//           └─► nvinfer (DashCamNet, FP16)
//               └─► nvtracker
//                   └─► nvdsosd  ◄── Pad Probe extracts NvDsObjectMeta here
//                       └─► fakesink
//
// The pad probe converts NvDsObjectMeta → DetBatch and pushes to an SPSC queue
// for consumption by the Stage E fusion thread (same interface as before).
//
// Strict constraint: this file is self-contained and does NOT touch any radar,
// IMU, or rear-camera code paths.
#pragma once

#include "adas/common/Types.hpp"
#include "adas/queues/SPSCQueue.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

// GStreamer / DeepStream headers — only compiled on aarch64 Jetson targets.
#ifdef __aarch64__
#include <gst/gst.h>
#include <gstnvdsmeta.h>
#include <nvdsmeta.h>
#endif

namespace adas {

// ─────────────────────────────────────────────────────────────────────────────
//  Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the DeepStream front-camera pipeline.
/// All values are locked to the capstone hardware spec; only change here.
struct DeepStreamConfig {
    // --- Stream ---
    std::string device_path   = "/dev/video0"; ///< V4L2 device (from hw_map)
    int         width         = 1280;          ///< Sensor width  [px]
    int         height        = 720;           ///< Sensor height [px]
    int         fps_num       = 10;            ///< Numerator   of FPS fraction
    int         fps_den       = 1;             ///< Denominator of FPS fraction
    std::string pixel_format  = "YUY2";        ///< YUYV → nvv4l2 maps "YUY2"

    // --- Model (NVIDIA DashCamNet) ---
    std::string config_file   = "config/dashcamnet_pgie_config.txt";
    ///< nvinfer config file; model uri, precision, batch-size are set there.

    // --- ROI filter (40% centre region of 1280×720 frame) ---
    // The central 40% by area → width = √0.4 × W ≈ 0.632×W, same for height.
    // We use the exact pixel values:  40% of 1280 = 512 (centred → x=[384,895])
    //                                  40% of 720  = 288 (centred → y=[216,503])
    static constexpr int ROI_X_MIN = 384;
    static constexpr int ROI_X_MAX = 895;
    static constexpr int ROI_Y_MIN = 216;
    static constexpr int ROI_Y_MAX = 503;

    // --- DashCamNet class IDs ---
    // 0=Car, 1=Bicycle, 2=Person, 3=RoadSign (currently dropped per spec)
    // Flip the entry to `true` to re-enable a class without touching probe logic.
    static constexpr bool CLASS_ENABLED[4] = {
        true,   // 0 — Car
        true,   // 1 — Bicycle
        true,   // 2 — Person
        false,  // 3 — RoadSign  (disabled per Phase-3 spec; toggle here)
    };
};

// ─────────────────────────────────────────────────────────────────────────────
//  FrontCamDeepStream
// ─────────────────────────────────────────────────────────────────────────────

/// Owns the full GStreamer/DeepStream pipeline for the Front Camera.
///
/// Thread model: a single GLib main-loop thread drives the GStreamer pipeline.
/// The pad probe runs on the GStreamer streaming thread; it pushes DetBatch
/// objects into the provided SPSC output queue (same interface as the old
/// CameraPipeline) so Stage E can consume them without any code changes there.
class FrontCamDeepStream {
  public:
    /// @param config         Hardware + model configuration (see DeepStreamConfig)
    /// @param output_queue   Queue to Stage E (DetBatch) — non-owning reference
    explicit FrontCamDeepStream(const DeepStreamConfig& config,
                                SPSCQueue<DetBatch, 8>& output_queue);

    ~FrontCamDeepStream();

    // Non-copyable / non-movable (owns GStreamer state)
    FrontCamDeepStream(const FrontCamDeepStream&)            = delete;
    FrontCamDeepStream& operator=(const FrontCamDeepStream&) = delete;

    /// Build and start the GStreamer pipeline + GLib main loop thread.
    /// Returns false immediately if GStreamer init fails or device is missing.
    bool start();

    /// Signal the GLib main loop to quit, then join the thread.
    void stop();

    bool isRunning()  const { return running_.load(std::memory_order_relaxed); }
    bool isHealthy()  const { return healthy_.load(std::memory_order_relaxed); }

    uint64_t getFramesProcessed() const {
        return frames_processed_.load(std::memory_order_relaxed);
    }

  private:
    // ── GStreamer pipeline construction ──────────────────────────────────────
    bool buildPipeline();
    void teardownPipeline();

    // ── GLib main-loop thread ────────────────────────────────────────────────
    void mainLoopThread();

    // ── Pad probe (static — GStreamer C callback) ────────────────────────────
    // Declared static so it can be used as a plain function pointer.
    // `user_data` is always `this`.
#ifdef __aarch64__
    static GstPadProbeReturn osdSinkPadProbe(GstPad*        pad,
                                             GstPadProbeInfo* info,
                                             gpointer         user_data);
#endif

    // ── Members ──────────────────────────────────────────────────────────────
    DeepStreamConfig            config_;
    SPSCQueue<DetBatch, 8>&     output_queue_;

    std::thread                 loop_thread_;
    std::atomic<bool>           running_{false};
    std::atomic<bool>           healthy_{false};
    std::atomic<uint64_t>       frames_processed_{0};

#ifdef __aarch64__
    GstElement*  pipeline_   = nullptr;
    GstElement*  osd_        = nullptr;   ///< nvdsosd — pad probe attachment point
    GstElement*  tracker_    = nullptr;   ///< nvtracker — IOU tracker element
    GMainLoop*   main_loop_  = nullptr;
#endif
};

} // namespace adas
