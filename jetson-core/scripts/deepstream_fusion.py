#!/usr/bin/env python3
"""
deepstream_fusion.py — ADAS Jetson Fusion Engine
=================================================
Standalone pipeline for the Jetson Nano that:
  1. Runs a GStreamer/DeepStream pipeline (DashCamNet + nvtracker) via pyds
  2. Subscribes to Pi ZMQ PUSH sockets for IMU, BSD-L, BSD-R sensor data
  3. Reads the front OPS243-C radar over serial for Z_rad / V_rad
  4. Performs 1D-2D ground-plane sensor fusion (FCW gating logic)
  5. Displays a live OpenCV UI with bounding boxes, fusion state, BSD indicators,
     and pitch/roll readout

Usage:
  python3 deepstream_fusion.py --pi-ip 192.168.1.100 --device /dev/video0 \
      --config ~/dashcamnet/config_infer.txt \
      --tracker ~/dashcamnet/tracker_config.txt \
      --radar-port /dev/ttyACM0
"""

import argparse
import collections
import math
import os
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import zmq

# ── GStreamer / pyds (aarch64 Jetson only) ────────────────────────────────────
try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst, GLib
    import pyds
    HAS_DEEPSTREAM = True
except ImportError:
    HAS_DEEPSTREAM = False
    print("[WARN] pyds / GStreamer not available — camera inference stubbed")

# ─────────────────────────────────────────────────────────────────────────────
#  Pi Protocol  (matches PiProtocol.hpp)
# ─────────────────────────────────────────────────────────────────────────────
PI_MAGIC = 0x50493034
PI_VERSION = 0x0100

# Header: magic(4) version(2) msg_type(2) payload_size(4) pad(4) ts_ns(8) seq(4) res(4) = 32B
HEADER_FMT  = "<IHHIIQII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 32, f"Header must be 32 bytes, got {HEADER_SIZE}"

MSG_REAR_RADAR_L = 0x0002
MSG_REAR_RADAR_R = 0x0003
MSG_IMU          = 0x0004

IMU_PAYLOAD  = struct.Struct("<ff")   # pitch_rad, roll_rad
BSD_PAYLOAD  = struct.Struct("<B")    # presence (0/1)

def parse_header(data: bytes):
    """Returns (magic, version, msg_type, payload_size, ts_ns, seq) or None."""
    if len(data) < HEADER_SIZE:
        return None
    magic, ver, msg_type, payload_size, _pad, ts_ns, seq, _res = \
        struct.unpack_from(HEADER_FMT, data, 0)
    if magic != PI_MAGIC or ver != PI_VERSION:
        return None
    return msg_type, payload_size, ts_ns, seq

# ─────────────────────────────────────────────────────────────────────────────
#  Camera & Fusion constants  (calibrate per physical setup)
# ─────────────────────────────────────────────────────────────────────────────
FRAME_W     = 1280
FRAME_H     = 720
F_Y         = 829.0    # Vertical focal length  [px]  (from calibration)
C_X         = 640.0    # Principal point x      [px]
C_Y         = 360.0    # Principal point y      [px]
H_CAM_M     = 0.90     # Camera height above ground [m]

# Fusion gating thresholds
DIST_GATE_M  = 5.0     # |Z_cam - Z_rad| tolerance  [m]
VEL_GATE_MPS = 3.0     # |V_cam - V_rad| tolerance  [m/s]
Z_MIN_M      = 1.0     # minimum believable camera range
Z_MAX_M      = 50.0    # maximum believable camera range

# Radar FOV → pixel ROI (30° FOV at 1280×720)
ROI_X_MIN, ROI_X_MAX = 384, 895
ROI_Y_MIN, ROI_Y_MAX = 216, 503

# DashCamNet class map
CLASS_NAMES  = {0: "Car", 1: "Bicycle", 2: "Person", 3: "RoadSign"}
CLASS_COLORS = {0: (0,255,0), 1: (255,165,0), 2: (0,200,255), 3: (128,0,128)}

# ─────────────────────────────────────────────────────────────────────────────
#  Shared state (written by background threads, read by probe + UI thread)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Detection:
    box:   Tuple[float,float,float,float]  # x,y,w,h [px]
    cls:   int
    conf:  float
    obj_id: int

@dataclass
class FusedObject:
    det:     Detection
    z_cam:   Optional[float]
    v_cam:   Optional[float]
    in_roi:  bool
    fused:   bool
    z_rad:   Optional[float] = None
    v_rad:   Optional[float] = None
    ttc_s:   Optional[float] = None

class AdasState:
    """Thread-safe container for all sensor values and latest frame."""
    def __init__(self):
        self._lock   = threading.Lock()
        self.pitch   = 0.0
        self.roll    = 0.0
        self.bsd_l   = False
        self.bsd_r   = False
        self.radar_z: Optional[float] = None
        self.radar_v: Optional[float] = None
        # Set by probe; consumed by UI
        self._frame:    Optional[np.ndarray] = None
        self._dets:     List[Detection]      = []
        self._fused:    List[FusedObject]    = []
        self._frame_id: int = 0

    # ── Writers ──────────────────────────────────────────────────────────────
    def set_imu(self, pitch: float, roll: float):
        with self._lock:
            self.pitch = pitch
            self.roll  = roll

    def set_bsd(self, side: str, presence: int):
        with self._lock:
            if side == "L":
                self.bsd_l = bool(presence)
            else:
                self.bsd_r = bool(presence)

    def set_radar(self, z: Optional[float], v: Optional[float]):
        with self._lock:
            self.radar_z = z
            self.radar_v = v

    def put_results(self, frame: Optional[np.ndarray],
                    dets: List[Detection], fused: List[FusedObject]):
        with self._lock:
            self._frame   = frame
            self._dets    = dets
            self._fused   = fused
            self._frame_id += 1

    # ── Readers ──────────────────────────────────────────────────────────────
    def get_imu(self) -> Tuple[float, float]:
        with self._lock:
            return self.pitch, self.roll

    def get_radar(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self.radar_z, self.radar_v

    def get_bsd(self) -> Tuple[bool, bool]:
        with self._lock:
            return self.bsd_l, self.bsd_r

    def get_results(self) -> Tuple[Optional[np.ndarray], List[Detection],
                                   List[FusedObject], int]:
        with self._lock:
            return self._frame, self._dets, self._fused, self._frame_id

# ─────────────────────────────────────────────────────────────────────────────
#  1D-2D Sensor Fusion
# ─────────────────────────────────────────────────────────────────────────────
class GroundPlaneFusion:
    """Ground-plane 1D-2D fusion matching camera tracks to OPS243-C radar."""

    def __init__(self):
        # object_id → deque of (z_cam_m, t_ns)
        self._history: Dict[int, collections.deque] = {}

    def _estimate_z(self, v_px: float, pitch_rad: float) -> Optional[float]:
        """Bottom-centre pixel row → longitudinal range [m]."""
        alpha = math.atan2(v_px - C_Y, F_Y)
        angle = pitch_rad + alpha
        if angle <= 1e-4:
            return None
        z = H_CAM_M / math.tan(angle)
        return z if Z_MIN_M <= z <= Z_MAX_M else None

    def _estimate_v(self, obj_id: int, z_now: float, t_ns: int) -> Optional[float]:
        """Temporal velocity estimate [m/s]; negative = approaching."""
        hist = self._history.setdefault(obj_id, collections.deque(maxlen=8))
        hist.append((z_now, t_ns))
        if len(hist) < 2:
            return None
        dz = hist[-1][0] - hist[0][0]
        dt = (hist[-1][1] - hist[0][1]) * 1e-9
        return (dz / dt) if dt > 0.001 else None

    def prune(self, active_ids):
        for k in list(self._history):
            if k not in active_ids:
                del self._history[k]

    def fuse(self, dets: List[Detection],
             radar_z: Optional[float], radar_v: Optional[float],
             pitch_rad: float) -> List[FusedObject]:
        results: List[FusedObject] = []
        active_ids = {d.obj_id for d in dets}
        t_ns = time.time_ns()

        for det in dets:
            x, y, w, h = det.box
            u = x + w * 0.5    # horizontal centre
            v = y + h          # bottom edge → road contact point

            # Phase 2: camera distance
            z_cam = self._estimate_z(v, pitch_rad)

            # Phase 3: camera velocity
            v_cam: Optional[float] = None
            if z_cam is not None:
                v_cam = self._estimate_v(det.obj_id, z_cam, t_ns)
            else:
                self._estimate_v(det.obj_id, 0.0, t_ns)  # keep history ticking

            # Gate 1: spatial ROI
            in_roi = (ROI_X_MIN <= u <= ROI_X_MAX and
                      ROI_Y_MIN <= v <= ROI_Y_MAX)

            obj = FusedObject(det=det, z_cam=z_cam, v_cam=v_cam,
                              in_roi=in_roi, fused=False)

            # Phases 4+5: radar association
            if (radar_z is not None and radar_v is not None
                    and in_roi and z_cam is not None):
                # Gate 2: distance
                if abs(z_cam - radar_z) <= DIST_GATE_M:
                    # Gate 3: velocity (skip if v_cam not yet stable)
                    v_rad_norm = -radar_v  # OPS243-C: positive=approaching
                    vel_ok = (v_cam is None or
                              abs(v_cam - v_rad_norm) <= VEL_GATE_MPS)
                    if vel_ok:
                        ttc = (radar_z / radar_v
                               if radar_v > 0.1 else float("inf"))
                        obj.fused   = True
                        obj.z_rad   = radar_z
                        obj.v_rad   = radar_v
                        obj.ttc_s   = ttc

            results.append(obj)

        self.prune(active_ids)
        return results

# ─────────────────────────────────────────────────────────────────────────────
#  ZMQ subscriber threads
# ─────────────────────────────────────────────────────────────────────────────
def zmq_imu_thread(state: AdasState, pi_ip: str, stop: threading.Event):
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.RCVTIMEO, 500)
    sock.connect(f"tcp://{pi_ip}:5558")
    print(f"[IMU] Connected to tcp://{pi_ip}:5558")
    while not stop.is_set():
        try:
            data = sock.recv()
        except zmq.Again:
            continue
        hdr = parse_header(data)
        if hdr is None or hdr[0] != MSG_IMU:
            continue
        payload = data[HEADER_SIZE:]
        if len(payload) >= IMU_PAYLOAD.size:
            pitch, roll = IMU_PAYLOAD.unpack_from(payload)
            state.set_imu(pitch, roll)
    sock.close()

def zmq_bsd_thread(state: AdasState, pi_ip: str, port: int,
                   side: str, msg_type: int, stop: threading.Event):
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.RCVTIMEO, 500)
    sock.connect(f"tcp://{pi_ip}:{port}")
    print(f"[BSD-{side}] Connected to tcp://{pi_ip}:{port}")
    while not stop.is_set():
        try:
            data = sock.recv()
        except zmq.Again:
            continue
        hdr = parse_header(data)
        if hdr is None or hdr[0] != msg_type:
            continue
        payload = data[HEADER_SIZE:]
        if len(payload) >= BSD_PAYLOAD.size:
            (presence,) = BSD_PAYLOAD.unpack_from(payload)
            state.set_bsd(side, presence)
    sock.close()

# ─────────────────────────────────────────────────────────────────────────────
#  Front radar serial thread  (OPS243-C)
# ─────────────────────────────────────────────────────────────────────────────
def radar_serial_thread(state: AdasState, port: str, baud: int,
                        stop: threading.Event):
    """
    Reads OPS243-C serial output.
    Typical lines: '{Sv: -3.45}' (speed m/s) and '{Rn: 4.56}' (range m).
    Positive Sv = approaching per OPS243-C convention.
    """
    try:
        import serial
    except ImportError:
        print("[Radar] pyserial not installed — front radar disabled")
        return

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"[Radar] Failed to open {port}: {e}")
        return

    print(f"[Radar] OPS243-C on {port} @ {baud} baud")
    z_buf: Optional[float] = None
    v_buf: Optional[float] = None

    while not stop.is_set():
        raw = ser.readline().decode("ascii", errors="ignore").strip()
        if not raw:
            continue
        # Parse range: e.g. "{Rn: 4.56}" or "Rn: 4.56"
        if "Rn" in raw or "R:" in raw:
            try:
                z_buf = float(raw.split(":")[-1].strip().strip("}"))
            except ValueError:
                pass
        # Parse velocity: e.g. "{Sv: -3.45}" or "Sv: -3.45"
        if "Sv" in raw or "V:" in raw:
            try:
                v_buf = float(raw.split(":")[-1].strip().strip("}"))
            except ValueError:
                pass
        # Publish when we have both
        if z_buf is not None and v_buf is not None:
            state.set_radar(z_buf, v_buf)
            z_buf = None
            v_buf = None

    ser.close()

# ─────────────────────────────────────────────────────────────────────────────
#  DeepStream pipeline
# ─────────────────────────────────────────────────────────────────────────────
_fusion_engine = GroundPlaneFusion()
_global_state: Optional[AdasState] = None

def osd_sink_pad_buffer_probe(pad, info, u_data):
    """
    GStreamer pad probe on nvdsosd sink pad.
    Extracts NvDsObjectMeta bounding boxes, runs fusion, stores results.
    """
    state: AdasState = u_data
    gst_buf = info.get_buffer()
    if not gst_buf:
        return Gst.PadProbeReturn.OK

    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buf))
    if not batch_meta:
        return Gst.PadProbeReturn.OK

    dets: List[Detection] = []

    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        l_obj = frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                obj = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break

            cls = int(obj.class_id)
            if 0 <= cls <= 2:  # Car, Bicycle, Person — skip RoadSign (3)
                r = obj.rect_params
                dets.append(Detection(
                    box=(r.left, r.top, r.width, r.height),
                    cls=cls,
                    conf=float(obj.confidence),
                    obj_id=int(obj.object_id),
                ))

            try:
                l_obj = l_obj.next
            except StopIteration:
                break

        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    # Run fusion
    pitch, _ = state.get_imu()
    radar_z, radar_v = state.get_radar()
    fused = _fusion_engine.fuse(dets, radar_z, radar_v, pitch)

    # Frame will arrive via appsink; put None here — visualizer merges them
    state.put_results(None, dets, fused)
    return Gst.PadProbeReturn.OK


def appsink_on_new_sample(appsink, state: AdasState):
    """appsink new-sample signal handler — maps buffer to numpy BGR frame."""
    sample = appsink.emit("pull-sample")
    if sample is None:
        return Gst.FlowReturn.OK

    buf  = sample.get_buffer()
    caps = sample.get_caps()
    structure = caps.get_structure(0)
    w = structure.get_value("width")
    h = structure.get_value("height")

    success, map_info = buf.map(Gst.MapFlags.READ)
    if not success:
        return Gst.FlowReturn.OK

    frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape(h, w, 3).copy()
    buf.unmap(map_info)

    # Merge latest frame into state
    _, dets, fused, _ = state.get_results()
    state.put_results(frame, dets, fused)
    return Gst.FlowReturn.OK


def build_deepstream_pipeline(args, state: AdasState):
    """
    Constructs the GStreamer pipeline programmatically.
    Topology:
      v4l2src → capsfilter → nvv4l2decoder → nvstreammux
        → nvinfer → nvtracker → nvdsosd → nvvidconv
        → capsfilter(BGR) → videoconvert → capsfilter(BGR) → appsink
    Returns (pipeline, main_loop).
    """
    if not HAS_DEEPSTREAM:
        return None, None

    Gst.init(None)
    pipeline = Gst.Pipeline.new("adas-pipeline")

    def make(plugin, name):
        el = Gst.ElementFactory.make(plugin, name)
        if not el:
            raise RuntimeError(f"Cannot create GStreamer element: {plugin}")
        return el

    src       = make("v4l2src",          "src")
    cf_src    = make("capsfilter",        "cf_src")
    decoder   = make("nvv4l2decoder",     "decoder")
    mux       = make("nvstreammux",       "mux")
    pgie      = make("nvinfer",           "infer")
    tracker   = make("nvtracker",         "tracker")
    osd       = make("nvdsosd",           "osd")
    nvvidconv = make("nvvidconv",         "nvvidconv")
    cf_bgr    = make("capsfilter",        "cf_bgr")
    vidconv   = make("videoconvert",      "vidconv")
    cf_bgr2   = make("capsfilter",        "cf_bgr2")
    appsink   = make("appsink",           "appsink")

    # Configure
    src.set_property("device", args.device)
    cf_src.set_property("caps", Gst.Caps.from_string(
        f"video/x-raw,format=YUY2,width={FRAME_W},height={FRAME_H},"
        f"framerate={args.fps}/1"))
    mux.set_property("batch-size", 1)
    mux.set_property("width", FRAME_W)
    mux.set_property("height", FRAME_H)
    mux.set_property("batched-push-timeout", 4_000_000)
    mux.set_property("live-source", 1)
    pgie.set_property("config-file-path", args.config)
    tracker.set_property("tracker-width",  640)
    tracker.set_property("tracker-height", 384)
    tracker.set_property("ll-lib-file",
        "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so")
    tracker.set_property("ll-config-file", args.tracker)
    cf_bgr.set_property("caps", Gst.Caps.from_string(
        "video/x-raw(memory:NVMM),format=I420"))
    cf_bgr2.set_property("caps", Gst.Caps.from_string(
        "video/x-raw,format=BGR"))
    appsink.set_property("emit-signals", True)
    appsink.set_property("max-buffers",  1)
    appsink.set_property("drop",         True)
    appsink.set_property("sync",         False)

    # Add to pipeline
    for el in [src, cf_src, decoder, mux, pgie, tracker,
               osd, nvvidconv, cf_bgr, vidconv, cf_bgr2, appsink]:
        pipeline.add(el)

    # Link: src → cf_src → decoder (mux has request sink pad)
    src.link(cf_src)
    cf_src.link(decoder)

    dec_src  = decoder.get_static_pad("src")
    mux_sink = mux.get_request_pad("sink_0")
    if not dec_src or not mux_sink:
        raise RuntimeError("Could not get decoder src / mux sink_0 pad")
    dec_src.link(mux_sink)

    # mux → infer → tracker → osd → vidconv chain → appsink
    for a, b in [(mux, pgie), (pgie, tracker), (tracker, osd),
                 (osd, nvvidconv), (nvvidconv, cf_bgr),
                 (cf_bgr, vidconv), (vidconv, cf_bgr2), (cf_bgr2, appsink)]:
        if not a.link(b):
            raise RuntimeError(f"Failed to link {a.get_name()} → {b.get_name()}")

    # Pad probe on nvdsosd sink pad
    osd_sink = osd.get_static_pad("sink")
    osd_sink.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, state)

    # appsink new-sample signal
    appsink.connect("new-sample", appsink_on_new_sample, state)

    main_loop = GLib.MainLoop()
    return pipeline, main_loop

# ─────────────────────────────────────────────────────────────────────────────
#  OpenCV Visualizer
# ─────────────────────────────────────────────────────────────────────────────
_FONT      = cv2.FONT_HERSHEY_SIMPLEX
_FUSED_CLR = (0, 0, 255)    # red — FCW candidate
_NORMAL_CLR = (0, 255, 0)   # green — camera-only
_BSD_ON    = (0, 255, 255)   # yellow
_BSD_OFF   = (60, 60, 60)    # dark grey

def draw_frame(frame: np.ndarray, dets: List[Detection],
               fused: List[FusedObject], state: AdasState) -> np.ndarray:
    """Render all overlays onto frame; returns the annotated image."""
    canvas = frame.copy()
    pitch, roll = state.get_imu()
    bsd_l, bsd_r = state.get_bsd()

    fused_ids = {f.det.obj_id for f in fused if f.fused}

    # ── Bounding boxes ────────────────────────────────────────────────────────
    for fobj in fused:
        d   = fobj.det
        x, y, w, h = [int(v) for v in d.box]
        color = _FUSED_CLR if fobj.fused else CLASS_COLORS.get(d.cls, _NORMAL_CLR)
        cv2.rectangle(canvas, (x, y), (x+w, y+h), color, 2)

        label = CLASS_NAMES.get(d.cls, "?")
        if fobj.fused and fobj.v_rad is not None:
            label += f"  V={fobj.v_rad:+.1f}m/s"
        if fobj.fused and fobj.ttc_s is not None:
            ttc_str = f"{fobj.ttc_s:.1f}s" if fobj.ttc_s < 99 else "inf"
            label += f"  TTC={ttc_str}"
        cv2.putText(canvas, label, (x, y-8), _FONT, 0.55, color, 1, cv2.LINE_AA)

        # Z_cam readout
        if fobj.z_cam is not None:
            cv2.putText(canvas, f"Z={fobj.z_cam:.1f}m",
                        (x, y+h+16), _FONT, 0.45, (200,200,200), 1)

    # ── ROI bounding box ──────────────────────────────────────────────────────
    cv2.rectangle(canvas, (ROI_X_MIN, ROI_Y_MIN),
                  (ROI_X_MAX, ROI_Y_MAX), (255, 255, 0), 1)
    cv2.putText(canvas, "Radar FOV", (ROI_X_MIN+4, ROI_Y_MIN-6),
                _FONT, 0.4, (255,255,0), 1)

    # ── HUD: pitch / roll ─────────────────────────────────────────────────────
    cv2.putText(canvas,
                f"Pitch: {math.degrees(pitch):+.2f}  Roll: {math.degrees(roll):+.2f}",
                (10, 24), _FONT, 0.6, (255,255,255), 1, cv2.LINE_AA)

    # ── BSD indicators (circles, bottom-left / bottom-right) ─────────────────
    radius = 28
    margin = 50
    cy_bsd = FRAME_H - margin
    # Left
    cv2.circle(canvas, (margin, cy_bsd), radius,
               _BSD_ON if bsd_l else _BSD_OFF, -1)
    cv2.putText(canvas, "BSD-L", (margin-24, cy_bsd+radius+16),
                _FONT, 0.45, (200,200,200), 1)
    # Right
    cv2.circle(canvas, (FRAME_W - margin, cy_bsd), radius,
               _BSD_ON if bsd_r else _BSD_OFF, -1)
    cv2.putText(canvas, "BSD-R", (FRAME_W-margin-24, cy_bsd+radius+16),
                _FONT, 0.45, (200,200,200), 1)

    # ── Detection list (top-right) ────────────────────────────────────────────
    x_list = FRAME_W - 240
    y_list = 20
    cv2.putText(canvas, f"Dets: {len(dets)}  Fused: {len(fused_ids)}",
                (x_list, y_list), _FONT, 0.5, (200,200,50), 1)
    for i, fobj in enumerate(fused[:12]):
        d = fobj.det
        marker = "[FCW]" if fobj.fused else "     "
        txt = f"{marker} {CLASS_NAMES.get(d.cls,'?')} {d.conf:.0%}"
        clr = _FUSED_CLR if fobj.fused else (180,180,180)
        cv2.putText(canvas, txt, (x_list, y_list + 18*(i+1)),
                    _FONT, 0.4, clr, 1)

    return canvas


def visualizer_loop(state: AdasState, stop: threading.Event):
    """Main thread — pulls frames from state, draws overlays, shows window."""
    cv2.namedWindow("ADAS Fusion", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ADAS Fusion", FRAME_W, FRAME_H)

    last_fid = -1
    placeholder = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Waiting for DeepStream frames...",
                (200, FRAME_H//2), _FONT, 0.9, (255,255,255), 2)

    while not stop.is_set():
        frame, dets, fused, fid = state.get_results()

        if frame is None or fid == last_fid:
            # No new frame — still draw HUD on placeholder
            canvas = draw_frame(placeholder.copy(), [], [], state)
        else:
            last_fid = fid
            canvas = draw_frame(frame, dets, fused, state)

        cv2.imshow("ADAS Fusion", canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            stop.set()
            break
        time.sleep(0.016)  # ~60 Hz poll

    cv2.destroyAllWindows()

# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="ADAS Jetson Fusion Engine")
    parser.add_argument("--pi-ip",      required=True,
                        help="IP of the Raspberry Pi 4 (ZMQ source)")
    parser.add_argument("--device",     default="/dev/video0",
                        help="V4L2 device for front camera (default: /dev/video0)")
    parser.add_argument("--config",
                        default=os.path.expanduser("~/dashcamnet/config_infer.txt"),
                        help="nvinfer config file (default: ~/dashcamnet/config_infer.txt)")
    parser.add_argument("--tracker",
                        default=os.path.expanduser("~/dashcamnet/tracker_config.txt"),
                        help="nvtracker config file")
    parser.add_argument("--fps",        type=int, default=10,
                        help="Camera FPS (default: 10)")
    parser.add_argument("--radar-port", default=None,
                        help="Serial port for OPS243-C (e.g. /dev/ttyACM0); "
                             "omit to disable front radar")
    parser.add_argument("--radar-baud", type=int, default=115200,
                        help="OPS243-C baud rate (default: 115200)")
    parser.add_argument("--verbose",    action="store_true")
    args = parser.parse_args()

    state = AdasState()
    stop  = threading.Event()

    # ── Signal handler ─────────────────────────────────────────────────────
    def _sig(sig, frame):
        print("\n[Main] Shutting down...")
        stop.set()
    signal.signal(signal.SIGINT,  _sig)
    signal.signal(signal.SIGTERM, _sig)

    # ── Background threads ─────────────────────────────────────────────────
    threads = [
        threading.Thread(target=zmq_imu_thread,
                         args=(state, args.pi_ip, stop), daemon=True),
        threading.Thread(target=zmq_bsd_thread,
                         args=(state, args.pi_ip, 5556, "L",
                               MSG_REAR_RADAR_L, stop), daemon=True),
        threading.Thread(target=zmq_bsd_thread,
                         args=(state, args.pi_ip, 5557, "R",
                               MSG_REAR_RADAR_R, stop), daemon=True),
    ]
    if args.radar_port:
        threads.append(threading.Thread(
            target=radar_serial_thread,
            args=(state, args.radar_port, args.radar_baud, stop),
            daemon=True))

    for t in threads:
        t.start()

    # ── DeepStream pipeline ────────────────────────────────────────────────
    pipeline = main_loop = None
    gst_thread = None
    if HAS_DEEPSTREAM:
        try:
            pipeline, main_loop = build_deepstream_pipeline(args, state)
            ret = pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("[DS] FAILED to set pipeline to PLAYING — check device/config paths")
                pipeline = None
            else:
                print(f"[DS] Pipeline started: {args.device} @ {args.fps} FPS")
                gst_thread = threading.Thread(
                    target=main_loop.run, daemon=True)
                gst_thread.start()
        except Exception as e:
            print(f"[DS] Pipeline build error: {e}")
            pipeline = None
    else:
        print("[DS] DeepStream not available — running ZMQ+Radar only")

    # ── Visualizer (blocks main thread) ───────────────────────────────────
    visualizer_loop(state, stop)

    # ── Cleanup ────────────────────────────────────────────────────────────
    if pipeline:
        pipeline.set_state(Gst.State.NULL)
    if main_loop and main_loop.is_running():
        main_loop.quit()
    for t in threads:
        t.join(timeout=2.0)
    print("[Main] Done.")


if __name__ == "__main__":
    main()
