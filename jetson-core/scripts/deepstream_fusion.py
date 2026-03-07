#!/usr/bin/env python3
"""
deepstream_fusion.py — ADAS Jetson Fusion Engine
=================================================
Architecture (strictly pyds / GStreamer — no custom TRT inference loop):

  v4l2src → nvv4l2decoder → nvstreammux
    → nvinfer (config-file-path = config_infer.txt)
    → nvtracker
    → nvdsosd   ◄──── buffer probe attached here (osd_sink_pad_buffer_probe)
    → nveglglessink (DeepStream owns all rendering)

The probe extracts NvDsObjectMeta bounding boxes, runs 1D-2D sensor fusion,
and injects display-meta lines back into the OSD so DeepStream renders the
fusion annotations (V_rad, TTC, BSD circles, pitch/roll) natively.

External inputs via ZMQ (Pi) and serial (OPS243-C front radar).

Usage:
  python3 deepstream_fusion.py \
      --pi-ip 192.168.55.2 \
      --device /dev/video0 \
      --config ~/dashcamnet/config_infer.txt \
      --tracker ~/dashcamnet/tracker_config.txt \
      [--radar-port /dev/ttyACM0] \
      [--no-display]   # fakesink instead of egl
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
from typing import Dict, List, Optional, Tuple

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

# DeepStream Python bindings (pyds.so) are not installed as a normal pip package.
# Add the DeepStream lib directory to sys.path so any python3 interpreter finds it.
_DS_PYDS_PATHS = [
    "/opt/nvidia/deepstream/deepstream/lib",
    "/opt/nvidia/deepstream/deepstream-6.0/lib",
    "/opt/nvidia/deepstream/deepstream-6.1/lib",
]
for _p in _DS_PYDS_PATHS:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
        break

try:
    import pyds
except ModuleNotFoundError:
    print("[DS] ERROR: pyds not found. Tried paths:")
    for _p in _DS_PYDS_PATHS:
        print(f"  {_p}")
    print("[DS] Install DeepStream 6.x or check PYTHONPATH.")
    sys.exit(1)

import zmq

# ─────────────────────────────────────────────────────────────────────────────
#  Pi Protocol  (matches PiProtocol.hpp exactly)
# ─────────────────────────────────────────────────────────────────────────────
PI_MAGIC    = 0x50493034
PI_VERSION  = 0x0100
HEADER_FMT  = "<IHHIIQII"   # 32 bytes: magic ver type size pad ts_ns seq res
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 32

MSG_REAR_RADAR_L = 0x0002
MSG_REAR_RADAR_R = 0x0003
MSG_IMU          = 0x0004

BSD_PAYLOAD = struct.Struct("<B")    # presence (0/1)
IMU_PAYLOAD = struct.Struct("<ff")   # pitch_rad, roll_rad

def _parse_header(data: bytes):
    """Returns (msg_type, payload_size) or None on invalid magic/version."""
    if len(data) < HEADER_SIZE:
        return None
    magic, ver, msg_type, payload_size, _pad, _ts, _seq, _res = \
        struct.unpack_from(HEADER_FMT, data)
    if magic != PI_MAGIC or ver != PI_VERSION:
        return None
    return msg_type, payload_size

# ─────────────────────────────────────────────────────────────────────────────
#  Camera calibration & fusion constants
# ─────────────────────────────────────────────────────────────────────────────
FRAME_W   = 1280
FRAME_H   = 720
F_Y       = 829.0   # vertical focal length [px]  — adjust to your calibration
C_Y       = 360.0   # principal point y    [px]
H_CAM_M   = 0.90    # camera height above road surface [m]

# Gating thresholds
DIST_GATE_M  = 5.0   # |Z_cam - Z_rad| [m]
VEL_GATE_MPS = 3.0   # |V_cam - V_rad| [m/s]
Z_MIN_M      = 1.5
Z_MAX_M      = 60.0

# Radar FOV → pixel ROI  (30° FOV, 1280×720)
ROI_X_MIN, ROI_X_MAX = 384, 895
ROI_Y_MIN, ROI_Y_MAX = 216, 503

# DashCamNet class IDs
CLASS_NAMES = {0: "Car", 1: "Bicycle", 2: "Person", 3: "RoadSign"}
# Class 3 (RoadSign) is dropped in the probe

# ─────────────────────────────────────────────────────────────────────────────
#  Thread-safe shared state
# ─────────────────────────────────────────────────────────────────────────────
class AdasState:
    def __init__(self):
        self._lock  = threading.Lock()
        self.pitch  = 0.0   # radians, from Pi ZMQ
        self.roll   = 0.0   # radians, from Pi ZMQ
        self.bsd_l  = False
        self.bsd_r  = False
        self.radar_z: Optional[float] = None  # range [m]
        self.radar_v: Optional[float] = None  # speed [m/s], positive=approaching

    def set_imu(self, pitch, roll):
        with self._lock: self.pitch = pitch; self.roll = roll

    def set_bsd(self, side, presence):
        with self._lock:
            if side == "L": self.bsd_l = bool(presence)
            else:            self.bsd_r = bool(presence)

    def set_radar(self, z, v):
        with self._lock: self.radar_z = z; self.radar_v = v

    def snapshot(self):
        """Return a consistent copy of all sensor values."""
        with self._lock:
            return (self.pitch, self.roll, self.bsd_l, self.bsd_r,
                    self.radar_z, self.radar_v)

# ─────────────────────────────────────────────────────────────────────────────
#  1D-2D sensor fusion (ground-plane model)
# ─────────────────────────────────────────────────────────────────────────────
class GroundPlaneFusion:
    def __init__(self):
        self._hist: Dict[int, collections.deque] = {}

    def estimate_z(self, v_bottom_px: float, pitch_rad: float) -> Optional[float]:
        """Convert bottom-edge pixel row to longitudinal range [m]."""
        alpha = math.atan2(v_bottom_px - C_Y, F_Y)
        angle = pitch_rad + alpha
        if angle <= 1e-4:
            return None
        z = H_CAM_M / math.tan(angle)
        return z if Z_MIN_M <= z <= Z_MAX_M else None

    def estimate_v(self, obj_id: int, z: float, t_ns: int) -> Optional[float]:
        """Temporal velocity [m/s]; negative = approaching."""
        buf = self._hist.setdefault(obj_id, collections.deque(maxlen=8))
        buf.append((z, t_ns))
        if len(buf) < 2:
            return None
        dz = buf[-1][0] - buf[0][0]
        dt = (buf[-1][1] - buf[0][1]) * 1e-9
        return dz / dt if dt > 0.001 else None

    def prune(self, active_ids):
        for k in list(self._hist):
            if k not in active_ids:
                del self._hist[k]

    def fuse(self, dets, radar_z, radar_v, pitch_rad):
        """
        dets: list of dicts with keys box(x,y,w,h), cls, conf, obj_id
        Returns list of result dicts with fusion fields added.
        """
        results = []
        active_ids = set()
        t_ns = time.time_ns()

        for d in dets:
            x, y, w, h = d["box"]
            u = x + w * 0.5   # horizontal centre
            v = y + h         # bottom edge — road contact point
            oid = d["obj_id"]
            active_ids.add(oid)

            z_cam = self.estimate_z(v, pitch_rad)
            v_cam = self.estimate_v(oid, z_cam if z_cam else 0.0, t_ns)

            in_roi = ROI_X_MIN <= u <= ROI_X_MAX and ROI_Y_MIN <= v <= ROI_Y_MAX

            r = dict(d)
            r.update(z_cam=z_cam, v_cam=v_cam, in_roi=in_roi,
                     fused=False, z_rad=None, v_rad=None, ttc=None)

            if (radar_z is not None and radar_v is not None
                    and in_roi and z_cam is not None):
                if abs(z_cam - radar_z) <= DIST_GATE_M:
                    # OPS243-C: positive velocity = approaching
                    vel_ok = (v_cam is None or
                              abs(v_cam - (-radar_v)) <= VEL_GATE_MPS)
                    if vel_ok:
                        ttc = (radar_z / radar_v) if radar_v > 0.1 else float("inf")
                        r.update(fused=True, z_rad=radar_z,
                                 v_rad=radar_v, ttc=ttc)

            results.append(r)

        self.prune(active_ids)
        return results

# ─────────────────────────────────────────────────────────────────────────────
#  ZMQ subscriber threads  (connect to Pi's PUSH sockets)
# ─────────────────────────────────────────────────────────────────────────────
def _zmq_imu_thread(state: AdasState, pi_ip: str, stop: threading.Event):
    ctx  = zmq.Context.instance()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.RCVTIMEO, 500)
    sock.connect(f"tcp://{pi_ip}:5558")
    print(f"[IMU ] Connected tcp://{pi_ip}:5558")
    while not stop.is_set():
        try:
            data = sock.recv()
        except zmq.Again:
            continue
        hdr = _parse_header(data)
        if hdr and hdr[0] == MSG_IMU:
            payload = data[HEADER_SIZE:]
            if len(payload) >= IMU_PAYLOAD.size:
                pitch, roll = IMU_PAYLOAD.unpack_from(payload)
                state.set_imu(pitch, roll)
    sock.close()


def _zmq_bsd_thread(state, pi_ip, port, side, msg_type, stop):
    ctx  = zmq.Context.instance()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.RCVTIMEO, 500)
    sock.connect(f"tcp://{pi_ip}:{port}")
    print(f"[BSD-{side}] Connected tcp://{pi_ip}:{port}")
    while not stop.is_set():
        try:
            data = sock.recv()
        except zmq.Again:
            continue
        hdr = _parse_header(data)
        if hdr and hdr[0] == msg_type:
            payload = data[HEADER_SIZE:]
            if len(payload) >= BSD_PAYLOAD.size:
                (presence,) = BSD_PAYLOAD.unpack_from(payload)
                state.set_bsd(side, presence)
    sock.close()

# ─────────────────────────────────────────────────────────────────────────────
#  Front radar serial thread  (OPS243-C on Jetson)
# ─────────────────────────────────────────────────────────────────────────────
def _radar_serial_thread(state: AdasState, port: str, baud: int,
                         stop: threading.Event):
    try:
        import serial
    except ImportError:
        print("[Radar] pyserial missing — front radar disabled")
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"[Radar] Cannot open {port}: {e}")
        return
    print(f"[Radar] OPS243-C on {port} @ {baud} baud")
    z_buf = v_buf = None
    while not stop.is_set():
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            continue
        try:
            if "Rn" in line or "R:" in line:
                z_buf = float(line.split(":")[-1].strip().strip("}"))
            if "Sv" in line or "V:" in line:
                v_buf = float(line.split(":")[-1].strip().strip("}"))
        except ValueError:
            pass
        if z_buf is not None and v_buf is not None:
            state.set_radar(z_buf, v_buf)
            z_buf = v_buf = None
    ser.close()

# ─────────────────────────────────────────────────────────────────────────────
#  The Buffer Probe  —  THE heart of the integration
# ─────────────────────────────────────────────────────────────────────────────
_fusion = GroundPlaneFusion()

def osd_sink_pad_buffer_probe(pad, info, u_data):
    """
    Attached to the SINK pad of nvdsosd.
    Runs every frame BEFORE OSD renders — we can both read object meta AND
    inject display meta (text/lines) that OSD will then draw on screen.

    u_data is the AdasState instance.
    """
    state: AdasState = u_data
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        return Gst.PadProbeReturn.OK

    # ── Step 1: get the DeepStream batch metadata from the GstBuffer ──────────
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    if not batch_meta:
        return Gst.PadProbeReturn.OK

    # ── Step 2: read latest sensor snapshot (lock-free round-trip) ───────────
    pitch, roll, bsd_l, bsd_r, radar_z, radar_v = state.snapshot()

    # ── Step 3: iterate frame → object meta, collect detections ──────────────
    dets = []
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        l_obj = frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break

            cls = int(obj_meta.class_id)
            if 0 <= cls <= 2:          # Car / Bicycle / Person  (drop RoadSign)
                r = obj_meta.rect_params   # NvOSD_RectParams
                dets.append({
                    "box":    (r.left, r.top, r.width, r.height),
                    "cls":    cls,
                    "conf":   float(obj_meta.confidence),
                    "obj_id": int(obj_meta.object_id),
                    # keep a reference so we can update text label in-place
                    "_obj_meta": obj_meta,
                })

            try:
                l_obj = l_obj.next
            except StopIteration:
                break

        # ── Step 4: run 1D-2D fusion ──────────────────────────────────────────
        fused_results = _fusion.fuse(dets, radar_z, radar_v, pitch)

        # ── Step 5: annotate object labels in-place (nvdsosd will render them)─
        for res in fused_results:
            obj_meta = res["_obj_meta"]
            name = CLASS_NAMES.get(res["cls"], "?")

            if res["fused"]:
                v_str  = f"{res['v_rad']:+.1f}m/s" if res["v_rad"] is not None else ""
                ttc_str = (f"TTC:{res['ttc']:.1f}s"
                           if res["ttc"] is not None and res["ttc"] < 99
                           else "")
                z_str  = f"Z:{res['z_rad']:.1f}m" if res["z_rad"] is not None else ""
                label  = f"[FCW] {name} {v_str} {z_str} {ttc_str}".strip()
                # Highlight fused boxes in red by overriding border colour
                obj_meta.rect_params.border_color.set(1.0, 0.0, 0.0, 1.0)
                obj_meta.rect_params.border_width = 4
            else:
                z_str  = f"{res['z_cam']:.1f}m" if res["z_cam"] else ""
                label  = f"{name} {res['conf']:.0%} {z_str}".strip()

            # Overwrite the tracker label that OSD will render above the box
            obj_meta.text_params.display_text = label
            obj_meta.text_params.font_params.font_size = 14

        # ── Step 6: inject HUD display meta (pitch/roll + BSD circles) ────────
        display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_lines = 0
        display_meta.num_circles = 0

        # Pitch / roll text line
        txt = display_meta.text_params[0]
        txt.display_text = (
            f"Pitch:{math.degrees(pitch):+.1f}°  "
            f"Roll:{math.degrees(roll):+.1f}°  "
            f"Radar: {'%.1fm' % radar_z if radar_z else '--'}"
        )
        txt.x_offset = 10
        txt.y_offset = 30
        txt.font_params.font_name = "Serif"
        txt.font_params.font_size = 16
        txt.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)
        txt.set_bg_clr = 1
        txt.text_bg_clr.set(0.0, 0.0, 0.0, 0.6)
        display_meta.num_labels = 1

        # BSD circles — bottom-left (L) and bottom-right (R)
        BSD_R_PX  = 30
        BSD_Y     = FRAME_H - 55
        colors_lr = [
            (bsd_l, 80),                 # Left  circle x
            (bsd_r, FRAME_W - 80),       # Right circle x
        ]
        for i, (active, cx) in enumerate(colors_lr):
            c = display_meta.circle_params[i]
            c.xc = cx
            c.yc = BSD_Y
            c.radius = BSD_R_PX
            if active:
                c.circle_color.set(0.0, 1.0, 1.0, 1.0)   # cyan = ALERT
            else:
                c.circle_color.set(0.25, 0.25, 0.25, 0.8) # dark grey = clear
            c.has_bg_color = 1
            c.bg_color.set(0.0, 0.0, 0.0, 0.0)
        display_meta.num_circles = 2

        # BSD labels
        for i, (label_txt, cx) in enumerate([("BSD-L", 57), ("BSD-R", FRAME_W-103)]):
            lt = display_meta.text_params[i + 1]
            lt.display_text = label_txt
            lt.x_offset = cx
            lt.y_offset = BSD_Y + BSD_R_PX + 5
            lt.font_params.font_name  = "Serif"
            lt.font_params.font_size  = 13
            lt.font_params.font_color.set(0.85, 0.85, 0.85, 1.0)
            lt.set_bg_clr = 0
        display_meta.num_labels = 3  # pitch line + 2 BSD labels

        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK

# ─────────────────────────────────────────────────────────────────────────────
#  GStreamer pipeline construction
# ─────────────────────────────────────────────────────────────────────────────
def build_pipeline(args):
    """
    Pipeline topology (as mandated by DeepStream pyds architecture):

      v4l2src → capsfilter → nvv4l2decoder
        → nvstreammux
        → nvinfer (config-file-path = args.config)
        → nvtracker
        → nvdsosd   ◄──── osd_sink_pad_buffer_probe attached here
        → (nv3dsink  OR  fakesink if --no-display)
    """
    Gst.init(None)

    def make(plugin, name):
        el = Gst.ElementFactory.make(plugin, name)
        if not el:
            raise RuntimeError(f"Could not create GStreamer element: {plugin!r}. "
                               f"Is DeepStream / GStreamer installed?")
        return el

    pipeline   = Gst.Pipeline.new("adas-pipeline")
    source     = make("v4l2src",        "src")
    caps_src   = make("capsfilter",     "caps_src")
    decoder    = make("nvv4l2decoder",  "decoder")
    streammux  = make("nvstreammux",    "mux")
    nvinfer    = make("nvinfer",        "infer")
    nvtracker  = make("nvtracker",      "tracker")
    nvosd      = make("nvdsosd",        "osd")

    if args.rtsp:
        nvvidconv = make("nvvideoconvert", "nvvideo-converter")
        encoder_caps = make("capsfilter", "encoder-caps")
        encoder_caps.set_property("caps", Gst.Caps.from_string("video/x-raw(memory:NVMM), format=I420"))
        encoder = make("nvv4l2h264enc", "h264-encoder")
        encoder.set_property("bitrate", 4000000)
        encoder.set_property('preset-level', 1)
        encoder.set_property('insert-sps-pps', 1)
        encoder.set_property('bufapi-version', 1)
        rtppay = make("rtph264pay", "rtppay")
        sink = make("udpsink", "udpsink")
        sink.set_property("host", "224.224.255.255")
        sink.set_property("port", 5400)
        sink.set_property("async", False)
        sink.set_property("sync", 1)
    else:
        sink       = make("fakesink" if args.no_display else "nv3dsink", "sink")

    # ── Configure elements ────────────────────────────────────────────────────
    source.set_property("device", args.device)

    caps_src.set_property("caps", Gst.Caps.from_string(
        f"video/x-raw,format=YUY2,width={FRAME_W},"
        f"height={FRAME_H},framerate={args.fps}/1"))

    # nvstreammux — batch single stream
    streammux.set_property("batch-size",            1)
    streammux.set_property("width",                 FRAME_W)
    streammux.set_property("height",                FRAME_H)
    streammux.set_property("batched-push-timeout",  4_000_000)
    streammux.set_property("live-source",           1)

    # nvinfer — DeepStream loads the engine internally from config_infer.txt
    nvinfer.set_property("config-file-path", args.config)

    # nvtracker
    nvtracker.set_property("tracker-width",  640)
    nvtracker.set_property("tracker-height", 384)
    nvtracker.set_property("ll-lib-file",
        "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so")
    nvtracker.set_property("ll-config-file", args.tracker)
    nvtracker.set_property("enable-past-frame", 1)

    if not args.no_display:
        sink.set_property("sync", False)

    # ── Add all elements to pipeline ──────────────────────────────────────────
    elements = [source, caps_src, decoder, streammux,
                nvinfer, nvtracker, nvosd]
    if args.rtsp:
        elements.extend([nvvidconv, encoder_caps, encoder, rtppay, sink])
    else:
        elements.append(sink)

    for el in elements:
        pipeline.add(el)

    # ── Link: src → caps → decoder → (mux via request pad) ───────
    if not source.link(caps_src):
        raise RuntimeError("Failed to link v4l2src → capsfilter")
    if not caps_src.link(decoder):
        raise RuntimeError("Failed to link capsfilter → nvv4l2decoder")

    # decoder src pad → nvstreammux sink_0 (request pad)
    dec_src_pad = decoder.get_static_pad("src")
    mux_sink_pad = streammux.get_request_pad("sink_0")
    if not dec_src_pad or not mux_sink_pad:
        raise RuntimeError("Could not get decoder src pad or mux sink_0 pad")
    if dec_src_pad.link(mux_sink_pad) != Gst.PadLinkReturn.OK:
        raise RuntimeError("Failed to link decoder → nvstreammux")

    # ── Link: mux → infer → tracker → osd → sink ─────────────────────────────
    link_pairs = [(streammux, nvinfer), (nvinfer, nvtracker), (nvtracker, nvosd)]
    if args.rtsp:
        link_pairs.extend([
            (nvosd, nvvidconv), (nvvidconv, encoder_caps), 
            (encoder_caps, encoder), (encoder, rtppay), (rtppay, sink)
        ])
    else:
        link_pairs.append((nvosd, sink))

    for a, b in link_pairs:
        if not a.link(b):
            raise RuntimeError(f"Failed to link {a.get_name()} → {b.get_name()}")

    # ── Attach the buffer probe to the OSD SINK pad ───────────────────────────
    # This is the canonical DeepStream Python probe hook:
    osd_sink_pad = nvosd.get_static_pad("sink")
    if not osd_sink_pad:
        raise RuntimeError("Cannot get nvdsosd sink pad")
    osd_sink_pad.add_probe(
        Gst.PadProbeType.BUFFER,
        osd_sink_pad_buffer_probe,
        _global_state,            # u_data passed to every probe callback
    )
    print("[DS] Probe attached to nvdsosd sink pad")

    return pipeline

# ─────────────────────────────────────────────────────────────────────────────
#  GLib bus message handler
# ─────────────────────────────────────────────────────────────────────────────
def _bus_call(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("[DS] End-of-stream")
        loop.quit()
    elif t == Gst.MessageType.WARNING:
        err, dbg = message.parse_warning()
        print(f"[DS] WARNING: {err}  ({dbg})")
    elif t == Gst.MessageType.ERROR:
        err, dbg = message.parse_error()
        print(f"[DS] ERROR: {err}  ({dbg})")
        loop.quit()
    return True

# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────
_global_state: Optional[AdasState] = None

def main():
    global _global_state

    parser = argparse.ArgumentParser(
        description="ADAS Jetson Fusion Engine (pyds / DeepStream)")
    parser.add_argument("--pi-ip",      required=True,
                        help="Pi 4 IP address (ZMQ source)")
    parser.add_argument("--device",     default="/dev/video0",
                        help="V4L2 front-camera device")
    parser.add_argument("--config",
                        default=os.path.expanduser("~/dashcamnet/config_infer.txt"),
                        help="nvinfer config file path  "
                             "(deepstream_app.txt also supported)")
    parser.add_argument("--tracker",
                        default=os.path.expanduser("~/dashcamnet/tracker_config.txt"),
                        help="nvtracker ll-config-file path")
    parser.add_argument("--fps",        type=int, default=10,
                        help="Camera frame rate (default: 10)")
    parser.add_argument("--radar-port", default=None,
                        help="Serial port for OPS243-C  (e.g. /dev/ttyACM0)")
    parser.add_argument("--radar-baud", type=int, default=115200)
    parser.add_argument("--no-display", action="store_true",
                        help="Use fakesink instead of nv3dsink "
                             "(headless / SSH mode)")
    parser.add_argument("--rtsp", action="store_true",
                        help="Start an RTSP server to stream the pipeline output")
    args = parser.parse_args()

    # ── Shared state ──────────────────────────────────────────────────────────
    state = AdasState()
    _global_state = state
    stop  = threading.Event()

    # ── Signal handler ────────────────────────────────────────────────────────
    def _sig(sig, _frame):
        print(f"\n[Main] Signal {sig} — shutting down")
        stop.set()
    signal.signal(signal.SIGINT,  _sig)
    signal.signal(signal.SIGTERM, _sig)

    # ── Background sensor threads ─────────────────────────────────────────────
    threads = [
        threading.Thread(target=_zmq_imu_thread,
                         args=(state, args.pi_ip, stop), daemon=True),
        threading.Thread(target=_zmq_bsd_thread,
                         args=(state, args.pi_ip, 5556, "L",
                               MSG_REAR_RADAR_L, stop), daemon=True),
        threading.Thread(target=_zmq_bsd_thread,
                         args=(state, args.pi_ip, 5557, "R",
                               MSG_REAR_RADAR_R, stop), daemon=True),
    ]
    if args.radar_port:
        threads.append(threading.Thread(
            target=_radar_serial_thread,
            args=(state, args.radar_port, args.radar_baud, stop),
            daemon=True))
    for t in threads:
        t.start()

    # ── Build DeepStream pipeline ─────────────────────────────────────────────
    try:
        pipeline = build_pipeline(args)
    except RuntimeError as e:
        print(f"[Main] Pipeline build failed: {e}")
        stop.set()
        for t in threads:
            t.join(timeout=1.0)
        sys.exit(1)

    # ── GLib main loop + GStreamer bus ────────────────────────────────────────
    loop = GLib.MainLoop()
    bus  = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", _bus_call, loop)

    # ── RTSP Server ───────────────────────────────────────────────────────────
    if args.rtsp:
        try:
            gi.require_version('GstRtspServer', '1.0')
            from gi.repository import GstRtspServer
        except (ValueError, ImportError):
            print("[DS] ERROR: GstRtspServer typelib not found. Cannot start RTSP stream.")
            sys.exit(1)
            
        server = GstRtspServer.RTSPServer.new()
        server.set_address("0.0.0.0")
        server.set_service("8554")
        factory = GstRtspServer.RTSPMediaFactory.new()
        factory.set_launch('( udpsrc name=pay0 port=5400 buffer-size=524288 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=(string)H264, payload=96 " )')
        factory.set_shared(True)
        server.get_mount_points().add_factory("/ds-test", factory)
        server.attach(None)
        print(f"\n[DS] *** RTSP STREAMING ENABLED ***")
        print(f"[DS] *** Connect VLC to: rtsp://<THIS_JETSON_IP>:8554/ds-test ***\n")

    # ── Start pipeline ────────────────────────────────────────────────────────
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("[Main] FAILED to set pipeline PLAYING. "
              "Check device path and config file.")
        pipeline.set_state(Gst.State.NULL)
        stop.set()
        sys.exit(1)

    print(f"[Main] Pipeline PLAYING  "
          f"device={args.device}  config={args.config}")
    print("[Main] Press Ctrl-C to stop\n")

    # ── Run until signal or EOS ───────────────────────────────────────────────
    # Run the GLib loop in main thread; a signal will set stop and quit the loop
    def _watch_stop():
        while not stop.is_set():
            time.sleep(0.2)
        loop.quit()

    watch_t = threading.Thread(target=_watch_stop, daemon=True)
    watch_t.start()

    try:
        loop.run()
    except KeyboardInterrupt:
        pass

    # ── Cleanup ───────────────────────────────────────────────────────────────
    print("[Main] Stopping pipeline...")
    pipeline.set_state(Gst.State.NULL)
    stop.set()
    for t in threads:
        t.join(timeout=2.0)
    print("[Main] Done.")


if __name__ == "__main__":
    main()
