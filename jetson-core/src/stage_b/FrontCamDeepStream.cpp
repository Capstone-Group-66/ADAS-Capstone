// File: src/stage_b/FrontCamDeepStream.cpp
// Stage B — DeepStream 6.0 Front Camera Pipeline (Implementation)
//
// Pipeline topology (aarch64 / Jetson only):
//
//   v4l2src  →  capsfilter (YUYV 1280×720 @ 10 FPS)
//            →  nvv4l2decoder             (NVDEC YUYV → NV12 in GPU memory)
//            →  nvstreammux               (batch_size=1, required by nvinfer)
//            →  nvinfer  [g_object_set]   (DashCamNet FP16, interval=0)
//            →  nvtracker [g_object_set]  (IOU tracker — persistent object IDs)
//            →  nvdsosd                   (pad probe extracts NvDsObjectMeta)
//            →  nvoverlaysink sync=false  (Jetson HDMI output / display)
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

FrontCamDeepStream::FrontCamDeepStream(const DeepStreamConfig &config,
                                       SPSCQueue<DetBatch, 8> &output_queue)
    : config_(config), output_queue_(output_queue) {
  // GStreamer is initialised once per process in main(); safe to call again.
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }
}

FrontCamDeepStream::~FrontCamDeepStream() { stop(); }

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
            << " (" << config_.width << "x" << config_.height << " @ "
            << config_.fps_num << " FPS, DashCamNet FP16)\n";
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
  std::cout << "[FrontCamDS] Pipeline stopped (" << frames_processed_.load()
            << " frames processed)\n";
}

// ── Pipeline Construction ────────────────────────────────────────────────────
// Elements are created individually so we can use g_object_set() to configure
// nvinfer and nvtracker precisely — avoids escaping issues in parse_launch
// string and makes each property explicit and auditable.

bool FrontCamDeepStream::buildPipeline() {

  // ── 1. Create all elements ────────────────────────────────────────────
  pipeline_ = gst_pipeline_new("adas-front-cam");
  GstElement *src = gst_element_factory_make("v4l2src", "src");
  GstElement *caps_filt = gst_element_factory_make("capsfilter", "caps_filt");
  GstElement *decoder = gst_element_factory_make("nvv4l2decoder", "decoder");
  GstElement *mux = gst_element_factory_make("nvstreammux", "mux");
  GstElement *infer = gst_element_factory_make("nvinfer", "infer");
  tracker_ = gst_element_factory_make("nvtracker", "tracker");
  osd_ = gst_element_factory_make("nvdsosd", "osd");
  GstElement *sink = gst_element_factory_make("nvoverlaysink", "sink");

  if (!pipeline_ || !src || !caps_filt || !decoder || !mux || !infer ||
      !tracker_ || !osd_ || !sink) {
    std::cerr
        << "[FrontCamDS] Failed to create one or more GStreamer elements.\n"
        << "  Ensure DeepStream 6.0 plugins are installed:\n"
        << "    sudo apt install deepstream-6.0\n";
    // Clean up whatever was allocated
    if (pipeline_)
      gst_object_unref(pipeline_);
    if (src)
      gst_object_unref(src);
    if (caps_filt)
      gst_object_unref(caps_filt);
    if (decoder)
      gst_object_unref(decoder);
    if (mux)
      gst_object_unref(mux);
    if (infer)
      gst_object_unref(infer);
    if (tracker_)
      gst_object_unref(tracker_);
    if (osd_)
      gst_object_unref(osd_);
    if (sink)
      gst_object_unref(sink);
    pipeline_ = nullptr;
    tracker_ = nullptr;
    osd_ = nullptr;
    return false;
  }

  // ── 2. Configure v4l2src ──────────────────────────────────────────────
  g_object_set(src, "device", config_.device_path.c_str(), nullptr);

  // ── 3. Configure capsfilter (YUYV 1280×720 @ 10 FPS) ─────────────────
  // GStreamer uses "YUY2" as the FOURCC for YUYV/YUV 4:2:2 packed.
  GstCaps *caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, config_.pixel_format.c_str(),
      "width", G_TYPE_INT, config_.width, "height", G_TYPE_INT, config_.height,
      "framerate", GST_TYPE_FRACTION, config_.fps_num, config_.fps_den,
      nullptr);
  g_object_set(caps_filt, "caps", caps, nullptr);
  gst_caps_unref(caps);

  // ── 4. Configure nvstreammux ─────────────────────────────────────────
  g_object_set(mux, "batch-size", 1, "width", config_.width, "height",
               config_.height, "batched-push-timeout", 4000000, // 4 ms timeout
               "live-source", TRUE, nullptr);

  // ── 5. Configure nvinfer (DashCamNet) via g_object_set ───────────────
  // The config file supplies: model engine, FP16 precision (network-mode=2),
  // interval=0 (every frame), cluster-mode=1 (hardware NMS), label file.
  // We do NOT inline ONNX paths here — the text file is the single source
  // of truth for the model.  You manage that file on the Jetson.
  g_object_set(infer, "config-file-path", config_.config_file.c_str(), nullptr);

  // ── 6. Configure nvtracker (low-overhead IOU tracker) ────────────────
  // Using the NvMultiObjectTracker shared library in IOU mode.
  // ll-config-file: user-supplied tracker config (created by user on Jetson).
  // tracker-width/height: internal tracking resolution — 640×384 is a
  // good balance for a Nano; lower = faster, but coarser ID matching.
  g_object_set(
      tracker_, "tracker-width", 640, "tracker-height", 384, "ll-lib-file",
      "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so",
      "ll-config-file", "/home/capstone-66/dashcamnet/tracker_config.txt",
      nullptr);

  // ── 7. Configure nvoverlaysink (Jetson HDMI display, non-blocking) ──
  g_object_set(sink, "sync", FALSE, nullptr);

  // ── 8. Add all elements to the pipeline ──────────────────────────────
  gst_bin_add_many(GST_BIN(pipeline_), src, caps_filt, decoder, mux, infer,
                   tracker_, osd_, sink, nullptr);

  // ── 9. Link elements in order ────────────────────────────────────────
  // nvstreammux requires linking via a request sink pad.
  if (!gst_element_link_many(src, caps_filt, decoder, nullptr) ||
      !gst_element_link_many(infer, tracker_, osd_, sink, nullptr)) {
    std::cerr << "[FrontCamDS] Element linking failed\n";
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    tracker_ = nullptr;
    osd_ = nullptr;
    return false;
  }

  // Link decoder → mux via request sink pad
  GstPad *decoder_src_pad = gst_element_get_static_pad(decoder, "src");
  GstPad *mux_sink_pad = gst_element_get_request_pad(mux, "sink_0");
  if (!decoder_src_pad || !mux_sink_pad ||
      gst_pad_link(decoder_src_pad, mux_sink_pad) != GST_PAD_LINK_OK) {
    std::cerr << "[FrontCamDS] Failed to link decoder → nvstreammux\n";
    if (decoder_src_pad)
      gst_object_unref(decoder_src_pad);
    if (mux_sink_pad)
      gst_object_unref(mux_sink_pad);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    tracker_ = nullptr;
    osd_ = nullptr;
    return false;
  }
  gst_object_unref(decoder_src_pad);
  gst_object_unref(mux_sink_pad);

  // Link mux → infer
  if (!gst_element_link(mux, infer)) {
    std::cerr << "[FrontCamDS] Failed to link nvstreammux → nvinfer\n";
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    tracker_ = nullptr;
    osd_ = nullptr;
    return false;
  }

  // ── 10. Attach pad probe to nvdsosd sink pad ──────────────────────────
  // The probe fires on every buffer after nvtracker has stamped object_id,
  // giving us stable IDs for the radar–camera fusion in Stage E.
  GstPad *osd_sink_pad = gst_element_get_static_pad(osd_, "sink");
  if (!osd_sink_pad) {
    std::cerr << "[FrontCamDS] Could not get osd sink pad\n";
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    tracker_ = nullptr;
    osd_ = nullptr;
    return false;
  }
  gst_pad_add_probe(osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                    &FrontCamDeepStream::osdSinkPadProbe,
                    this,     // user_data → `this`
                    nullptr); // GDestroyNotify
  gst_object_unref(osd_sink_pad);

  // ── 11. Create GLib main loop and set pipeline to PLAYING ────────────
  main_loop_ = g_main_loop_new(nullptr, FALSE);

  GstStateChangeReturn ret =
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "[FrontCamDS] Failed to set pipeline to PLAYING\n";
    g_main_loop_unref(main_loop_);
    main_loop_ = nullptr;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    tracker_ = nullptr;
    osd_ = nullptr;
    return false;
  }

  std::cout
      << "[FrontCamDS] Pipeline built:\n"
      << "  src    : " << config_.device_path << "  (" << config_.pixel_format
      << " " << config_.width << "x" << config_.height << " @ "
      << config_.fps_num << " FPS)\n"
      << "  model  : " << config_.config_file << "  (DashCamNet FP16)\n"
      << "  tracker: IOU — /home/capstone-66/dashcamnet/tracker_config.txt\n"
      << "  sink   : nvoverlaysink (sync=false)\n";
  return true;
}

void FrontCamDeepStream::teardownPipeline() {
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    // osd_ and tracker_ are owned by the pipeline bin; unref our handles.
    if (osd_) {
      gst_object_unref(osd_);
      osd_ = nullptr;
    }
    if (tracker_) {
      gst_object_unref(tracker_);
      tracker_ = nullptr;
    }
    gst_object_unref(pipeline_);
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
GstPadProbeReturn FrontCamDeepStream::osdSinkPadProbe(GstPad * /*pad*/,
                                                      GstPadProbeInfo *info,
                                                      gpointer user_data) {
  auto *self = static_cast<FrontCamDeepStream *>(user_data);

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  if (!buf) {
    return GST_PAD_PROBE_OK;
  }

  // ── A) Extract DeepStream batch metadata ─────────────────────────────
  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
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

  for (NvDsMetaList *fl = batch_meta->frame_meta_list; fl; fl = fl->next) {
    auto *frame_meta = static_cast<NvDsFrameMeta *>(fl->data);

    // Timestamp: use Jetson monotonic clock at probe invocation time.
    // This is the authoritative ingest timestamp; DeepStream's own
    // pts/dts is not used here because Clock::now_ns() is the pipeline's
    // canonical time source (consistent with all other sensors).
    const uint64_t t_ingest = Clock::now_ns();

    Header hdr;
    hdr.t_ingest_ns = t_ingest;
    hdr.mount = Mount::FrontCam;
    hdr.seq = static_cast<uint32_t>(
        self->frames_processed_.load(std::memory_order_relaxed));
    hdr.healthy = true;

    DetBatch batch;
    batch.h = hdr;
    // Note: batch.frame intentionally left empty — DeepStream frames live
    // in GPU NvBufSurface memory, not host cv::Mat. The visualiser in
    // main.cpp will treat an empty frame as "no display data" which is
    // safe — functionality is unaffected.

    for (NvDsMetaList *ol = frame_meta->obj_meta_list; ol; ol = ol->next) {
      auto *obj = static_cast<NvDsObjectMeta *>(ol->data);

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
      const NvBbox_Coords &bb = obj->detector_bbox_info.org_bbox_coords;
      const float cx = bb.left + bb.width * 0.5f;
      const float cy = bb.top + bb.height * 0.5f;

      if (cx < roi_x_min || cx > roi_x_max || cy < roi_y_min ||
          cy > roi_y_max) {
        continue; // Centre point outside 40% ROI — discard
      }

      // ── F) Convert to adas::Det and accumulate ────────────────
      // Det constructor: (box_px, class_id, confidence, object_id)
      // object_id is the persistent tracker ID assigned by nvtracker.
      // It is stable across frames so Stage E radar–camera fusion can
      // match the same physical object without re-running Hungarian.
      // Cast: NvDsObjectMeta::object_id is guint64 (== uint64_t).
      cv::Rect2f box(bb.left, bb.top, bb.width, bb.height);
      const uint64_t track_id = static_cast<uint64_t>(obj->object_id);
      batch.dets.emplace_back(box, cls, obj->confidence, track_id);
    }

    // Publish DetBatch (drop if queue full — freshness over completeness)
    self->output_queue_.try_push(std::move(batch));

    self->frames_processed_.fetch_add(1, std::memory_order_relaxed);
    self->healthy_.store(true, std::memory_order_relaxed);

    // Verbose mode logging (mirrors old CameraPipeline pattern)
    if (g_verbose_mode.load()) {
      static uint64_t verbose_frame_count = 0;
      static uint64_t verbose_det_count = 0;
      static auto verbose_last_time = std::chrono::steady_clock::now();
      ++verbose_frame_count;
      verbose_det_count += batch.dets.size();

      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         now - verbose_last_time)
                         .count();
      if (elapsed >= 5) {
        std::cout << "\n[Stage B / DeepStream] FrontCam: " << verbose_det_count
                  << " dets in " << verbose_frame_count << " frames\n"
                  << std::flush;
        verbose_frame_count = 0;
        verbose_det_count = 0;
        verbose_last_time = now;
      }
    }
  }

  return GST_PAD_PROBE_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
#else // !__aarch64__
// Stub implementation — non-Jetson developer machines
// ─────────────────────────────────────────────────────────────────────────────

FrontCamDeepStream::FrontCamDeepStream(const DeepStreamConfig &config,
                                       SPSCQueue<DetBatch, 8> &output_queue)
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
