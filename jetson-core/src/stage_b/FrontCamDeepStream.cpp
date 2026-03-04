// File: src/stage_b/FrontCamDeepStream.cpp
// Stage B — DeepStream 6.0 Front Camera Pipeline (Implementation)
//
// Pipeline topology (aarch64 / Jetson only):
//
//   v4l2src  →  capsfilter (YUYV 1280×720 @ 10 FPS)
//            →  nvv4l2decoder          (NVDEC YUYV → NV12 in GPU memory)
//            →  nvstreammux            (batch_size=1, required by nvinfer)
//            →  nvinfer                (DashCamNet, FP16, interval=0)
//            →  nvtracker              (IOU tracker — keeps obj_id stable)
//            →  nvdsosd                (draws metadata; OSD sink ← pad probe)
//            →  fakesink               (no display; pipeline is headless)
//
// The pad probe on the nvdsosd sink pad:
//   1. Iterates all NvDsBatchMeta → NvDsFrameMeta → NvDsObjectMeta entries.
//   2. Filters by class ID (class 3 / RoadSign dropped by default).
//   3. Applies a 40% centre-ROI spatial filter.
//   4. Converts surviving detections to adas::DetBatch and try_pushes to queue.
//
// On non-aarch64 targets the entire implementation is a no-op stub so the
// project can still be compiled and type-checked on developer laptops.
#include "adas/stage_b/FrontCamDeepStream.hpp"
#include "adas/common/Clock.hpp"
#include "adas/common/Globals.hpp"

#include <iostream>

namespace adas {

// ─────────────────────────────────────────────────────────────────────────────
#ifdef __aarch64__
// Full DeepStream implementation — Jetson / aarch64 only
// ─────────────────────────────────────────────────────────────────────────────

// ── Constructor / Destructor ─────────────────────────────────────────────────

FrontCamDeepStream::FrontCamDeepStream(const DeepStreamConfig& config,
                                       SPSCQueue<DetBatch, 8>& output_queue)
    : config_(config), output_queue_(output_queue) {
    // GStreamer is initialised once per process in main(); safe to call again.
    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }
}

FrontCamDeepStream::~FrontCamDeepStream() {
    stop();
}

// ── Public API ───────────────────────────────────────────────────────────────

bool FrontCamDeepStream::start() {
    if (running_.load(std::memory_order_relaxed)) {
        return true;
    }

    if (!buildPipeline()) {
        std::cerr << "[FrontCamDS] ERROR: Failed to build GStreamer pipeline\n";
        return false;
    }

    running_.store(true, std::memory_order_relaxed);
    loop_thread_ = std::thread(&FrontCamDeepStream::mainLoopThread, this);

    std::cout << "[FrontCamDS] Pipeline started on " << config_.device_path
              << " (" << config_.width << "x" << config_.height
              << " @ " << config_.fps_num << " FPS, DashCamNet FP16)\n";
    return true;
}

void FrontCamDeepStream::stop() {
    if (!running_.load(std::memory_order_relaxed)) {
        return;
    }

    running_.store(false, std::memory_order_relaxed);
    healthy_.store(false, std::memory_order_relaxed);

    if (main_loop_ && g_main_loop_is_running(main_loop_)) {
        g_main_loop_quit(main_loop_);
    }

    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }

    teardownPipeline();
    std::cout << "[FrontCamDS] Pipeline stopped ("
              << frames_processed_.load() << " frames processed)\n";
}

// ── Pipeline Construction ────────────────────────────────────────────────────

bool FrontCamDeepStream::buildPipeline() {
    GError* err = nullptr;

    // ── Capsfilter string for YUYV 1280×720 @ 10 FPS ─────────────────────
    // GStreamer uses "YUY2" as the FOURCC for YUYV / YUV 4:2:2 packed.
    std::string caps_str =
        "video/x-raw,format=" + config_.pixel_format +
        ",width="  + std::to_string(config_.width) +
        ",height=" + std::to_string(config_.height) +
        ",framerate=" + std::to_string(config_.fps_num) +
        "/"         + std::to_string(config_.fps_den);

    // ── Build the pipeline using gst_parse_launch for clarity ────────────
    // nvv4l2decoder: hardware-accelerated YUYV→NV12 conversion on NVDEC.
    // nvstreammux : mandatory before nvinfer; batch_size=1, live-source=1.
    // nvinfer     : reads model config from config_.config_file.
    //               The config file encodes: model-uri, precision=FP16,
    //               interval=0, cluster-mode=1 (hardware NMS).
    // nvtracker   : IOU tracker; keeps object IDs stable across frames.
    // nvdsosd     : renders OSD metadata; we attach our pad probe here.
    // fakesink    : we don't need display; all data exits via probe.
    std::string pipeline_desc =
        "v4l2src device=" + config_.device_path + " ! "
        "capsfilter caps=\"" + caps_str + "\" ! "
        "nvv4l2decoder ! "
        "nvstreammux name=mux batch-size=1 width=" + std::to_string(config_.width) +
            " height=" + std::to_string(config_.height) +
            " batched-push-timeout=4000000 live-source=1 ! "
        "nvinfer config-file-path=" + config_.config_file + " ! "
        "nvtracker tracker-width=640 tracker-height=384 "
            "ll-lib-file=/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so ! "
        "nvdsosd name=osd ! "
        "fakesink sync=false";

    pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &err);
    if (!pipeline_ || err) {
        std::cerr << "[FrontCamDS] gst_parse_launch error: "
                  << (err ? err->message : "unknown") << "\n";
        if (err) g_error_free(err);
        return false;
    }

    // ── Attach pad probe to nvdsosd sink pad ─────────────────────────────
    osd_ = gst_bin_get_by_name(GST_BIN(pipeline_), "osd");
    if (!osd_) {
        std::cerr << "[FrontCamDS] Could not find 'osd' element in pipeline\n";
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    GstPad* osd_sink_pad = gst_element_get_static_pad(osd_, "sink");
    if (!osd_sink_pad) {
        std::cerr << "[FrontCamDS] Could not get osd sink pad\n";
        gst_object_unref(osd_);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        osd_ = nullptr;
        return false;
    }

    // Install BUFFER probe: fires on every buffer (every frame) reaching OSD.
    gst_pad_add_probe(osd_sink_pad,
                      GST_PAD_PROBE_TYPE_BUFFER,
                      &FrontCamDeepStream::osdSinkPadProbe,
                      this,           // user_data → `this`
                      nullptr);       // GDestroyNotify
    gst_object_unref(osd_sink_pad);

    // ── Create GLib main loop ─────────────────────────────────────────────
    main_loop_ = g_main_loop_new(nullptr, FALSE);

    // ── Set pipeline to PLAYING ───────────────────────────────────────────
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[FrontCamDS] Failed to set pipeline to PLAYING\n";
        g_main_loop_unref(main_loop_);
        main_loop_ = nullptr;
        gst_object_unref(osd_);
        gst_object_unref(pipeline_);
        osd_ = nullptr;
        pipeline_ = nullptr;
        return false;
    }

    return true;
}

void FrontCamDeepStream::teardownPipeline() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(osd_);
        gst_object_unref(pipeline_);
        osd_     = nullptr;
        pipeline_ = nullptr;
    }
    if (main_loop_) {
        g_main_loop_unref(main_loop_);
        main_loop_ = nullptr;
    }
}

// ── GLib Main Loop Thread ────────────────────────────────────────────────────

void FrontCamDeepStream::mainLoopThread() {
    std::cout << "[FrontCamDS] GLib main loop thread started\n";

    // g_main_loop_run blocks until g_main_loop_quit() is called from stop().
    g_main_loop_run(main_loop_);

    std::cout << "[FrontCamDS] GLib main loop thread exited\n";
}

// ── Pad Probe: NvDsObjectMeta → DetBatch ─────────────────────────────────────

/// Static pad probe callback called by the GStreamer streaming thread.
///
/// Probe order of operations:
///   A) Extract batch metadata from the GstBuffer.
///   B) [RAMI'S LANE DETECTION WORKSPACE] — reserved for teammate.
///   C) Iterate frame / object metadata entries.
///   D) Filter by class ID (class 3 dropped per spec).
///   E) Apply 40% centre-ROI filter.
///   F) Build DetBatch and try_push to output queue.
GstPadProbeReturn
FrontCamDeepStream::osdSinkPadProbe(GstPad*         /*pad*/,
                                    GstPadProbeInfo* info,
                                    gpointer         user_data) {
    auto* self = static_cast<FrontCamDeepStream*>(user_data);

    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf) {
        return GST_PAD_PROBE_OK;
    }

    // ── A) Extract DeepStream batch metadata ─────────────────────────────
    NvDsBatchMeta* batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    if (!batch_meta) {
        return GST_PAD_PROBE_OK;
    }

    // ── B) RAMI'S LANE DETECTION WORKSPACE ───────────────────────────────
    //
    // When you are ready to integrate lane detection, this is where you
    // extract the raw frame surface from GPU memory and hand it off.
    //
    // Typical pattern (requires nvbufsurface.h):
    //
    //   NvBufSurface* surface = nullptr;
    //   gst_buffer_map(buf, &map_info, GST_MAP_READ);
    //   surface = reinterpret_cast<NvBufSurface*>(map_info.data);
    //   // surface->surfaceList[0] → NvBufSurfaceParams for frame 0
    //   // Map to CPU:  NvBufSurfaceMap(surface, 0, 0, NVBUF_MAP_READ);
    //   // Wrap as cv::Mat for lane-detection algorithm (NV12 → BGR if needed).
    //   // NvBufSurfaceUnmap / gst_buffer_unmap when done.
    //
    // ─────────────────────────────────────────────────────────────────────

    // ── C–F) Iterate frame & object metadata ──────────────────────────────
    // ROI constants (40% centre of 1280×720):
    //   x ∈ [384, 895]  (512 px wide,  centred on 640)
    //   y ∈ [216, 503]  (288 px high,  centred on 360)
    const int roi_x_min = DeepStreamConfig::ROI_X_MIN;
    const int roi_x_max = DeepStreamConfig::ROI_X_MAX;
    const int roi_y_min = DeepStreamConfig::ROI_Y_MIN;
    const int roi_y_max = DeepStreamConfig::ROI_Y_MAX;

    for (NvDsMetaList* fl = batch_meta->frame_meta_list; fl; fl = fl->next) {
        auto* frame_meta = static_cast<NvDsFrameMeta*>(fl->data);

        // Timestamp: use Jetson monotonic clock at probe invocation time.
        // This is the authoritative ingest timestamp; DeepStream's own
        // pts/dts is not used here because Clock::now_ns() is the pipeline's
        // canonical time source (consistent with all other sensors).
        const uint64_t t_ingest = Clock::now_ns();

        Header hdr;
        hdr.t_ingest_ns = t_ingest;
        hdr.mount       = Mount::FrontCam;
        hdr.seq         = static_cast<uint32_t>(
                              self->frames_processed_.load(std::memory_order_relaxed));
        hdr.valid       = true;

        DetBatch batch;
        batch.h = hdr;
        // Note: batch.frame intentionally left empty — DeepStream frames live
        // in GPU NvBufSurface memory, not host cv::Mat. The visualiser in
        // main.cpp will treat an empty frame as "no display data" which is
        // safe — functionality is unaffected.

        for (NvDsMetaList* ol = frame_meta->obj_meta_list; ol; ol = ol->next) {
            auto* obj = static_cast<NvDsObjectMeta*>(ol->data);

            // ── D) Class ID filter ─────────────────────────────────────
            // DashCamNet class map:  0=Car  1=Bicycle  2=Person  3=RoadSign
            // CLASS_ENABLED[] controls which classes are passed through.
            // To re-enable RoadSign later: set CLASS_ENABLED[3] = true in
            // FrontCamDeepStream.hpp — no code changes needed here.
            const int cls = static_cast<int>(obj->class_id);
            constexpr int NUM_DASHCAMNET_CLASSES = 4;
            if (cls < 0 || cls >= NUM_DASHCAMNET_CLASSES) {
                continue; // Unexpected class — skip silently
            }
            if (!DeepStreamConfig::CLASS_ENABLED[cls]) {
                continue; // Class filtered out by config table
            }

            // ── E) 40% Centre-ROI filter ──────────────────────────────
            // Compute bounding-box centre point from NvDsObjectMeta rect.
            // NvDsComp_BboxInfo::org_bbox_coords uses (left, top, width, height).
            const NvBbox_Coords& bb = obj->detector_bbox_info.org_bbox_coords;
            const float cx = bb.left + bb.width  * 0.5f;
            const float cy = bb.top  + bb.height * 0.5f;

            if (cx < roi_x_min || cx > roi_x_max ||
                cy < roi_y_min || cy > roi_y_max) {
                continue; // Centre point outside 40% ROI — discard
            }

            // ── F) Convert to adas::Det and accumulate ────────────────
            // Det constructor: (box_px, class_id, confidence_score)
            cv::Rect2f box(bb.left, bb.top, bb.width, bb.height);
            batch.dets.emplace_back(box,
                                    cls,
                                    obj->confidence);
        }

        // Publish DetBatch (drop if queue full — freshness over completeness)
        self->output_queue_.try_push(std::move(batch));

        self->frames_processed_.fetch_add(1, std::memory_order_relaxed);
        self->healthy_.store(true, std::memory_order_relaxed);

        // Verbose mode logging (mirrors old CameraPipeline pattern)
        if (g_verbose_mode.load()) {
            static uint64_t verbose_frame_count  = 0;
            static uint64_t verbose_det_count    = 0;
            static auto     verbose_last_time    = std::chrono::steady_clock::now();
            ++verbose_frame_count;
            verbose_det_count += batch.dets.size();

            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                               now - verbose_last_time).count();
            if (elapsed >= 5) {
                std::cout << "\n[Stage B / DeepStream] FrontCam: "
                          << verbose_det_count << " dets in "
                          << verbose_frame_count << " frames\n" << std::flush;
                verbose_frame_count = 0;
                verbose_det_count   = 0;
                verbose_last_time   = now;
            }
        }
    }

    return GST_PAD_PROBE_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
#else // !__aarch64__
// Stub implementation — non-Jetson developer machines
// ─────────────────────────────────────────────────────────────────────────────

FrontCamDeepStream::FrontCamDeepStream(const DeepStreamConfig& config,
                                       SPSCQueue<DetBatch, 8>& output_queue)
    : config_(config), output_queue_(output_queue) {
    std::cout << "[FrontCamDS] STUB: DeepStream not available on non-aarch64. "
                 "No-op pipeline created.\n";
}

FrontCamDeepStream::~FrontCamDeepStream() { stop(); }

bool FrontCamDeepStream::start() {
    std::cout << "[FrontCamDS] STUB: start() called — no-op on this platform\n";
    running_.store(false, std::memory_order_relaxed);
    return false; // Not a fatal error on dev machines; just nothing runs
}

void FrontCamDeepStream::stop() {
    running_.store(false, std::memory_order_relaxed);
    healthy_.store(false, std::memory_order_relaxed);
}

#endif // __aarch64__

} // namespace adas
