"""
laneDepartureWarning.py — AI-based Lane Departure Warning (LDW) System
Based on Ultra-Fast Lane Detection V2 (UFLD V2) with ONNX Runtime GPU inference.

Implements the LDW logic from the ADAS project report:
  - FR21: Issue LDW only when the vehicle breaches lane boundaries
  - FR22: Post "Lane Unavailable" when quality is below threshold for 10 frames
  - Lane Lock Confirmation: 10 consecutive frames before LDW is armed
  - Ego Speed Gate: LDW disabled below 20 km/h (simulated from config if no IMU)
  - Departure Geometry: point-polygon test against vehicle projection polygon
  - 2-second cooldown after each LDW event

Usage:
    python laneDepartureWarning.py --video temp/dashcam1.mp4 --model culane
    python laneDepartureWarning.py --video temp/dashcam1.mp4 --model tusimple --speed 80
    python laneDepartureWarning.py --video temp/dashcam1.mp4 --model culane --no-display
"""

import argparse
import time
import cv2
import numpy as np
from collections import deque

from ufldDetector.utils import LaneModelType
from ufldDetector.ultrafastLaneDetector import UltrafastLaneDetector
from ufldDetector.ultrafastLaneDetectorV2 import UltrafastLaneDetectorV2

# ── Constants ─────────────────────────────────────────────────────────────────

LANE_LOCK_THRESH    = 10      # consecutive frames required to arm LDW (FR21)
UNAVAILABLE_THRESH  = 10      # consecutive low-quality frames → "Lane Unavailable" (FR22)
LDW_COOLDOWN_SEC    = 2.0     # seconds between LDW alerts (FR21)
MIN_SPEED_KMH       = 20.0    # ego speed gate — LDW disabled below this (FR21)
QUALITY_THRESHOLD   = 0.3     # area_status proxy quality floor (FR22)

# Vehicle projection polygon — bottom-center trapezoid representing car's path
# Defined as fractions of (frame_width, frame_height), tuned for dashcam FOV
CAR_PATH_POLY_FRAC = np.array([
    [0.42, 1.00],   # bottom-left
    [0.58, 1.00],   # bottom-right
    [0.54, 0.60],   # top-right
    [0.46, 0.60],   # top-left
], dtype=np.float32)

# Colors (BGR)
COLOR_SAFE      = (46, 205, 46)     # green  — normal
COLOR_WARNING   = (0, 100, 255)     # orange — departure
COLOR_CRITICAL  = (0, 0, 255)       # red    — breach
COLOR_UNAVAIL   = (180, 180, 180)   # grey   — lane unavailable
COLOR_OVERLAY   = (0, 180, 255)     # amber  — path polygon
COLOR_LEFT      = (255, 80,  80)    # blue   — left lane
COLOR_RIGHT     = (80,  200, 80)    # green  — right lane

MODEL_MAP = {
    "culane":   ("models/culane_res18.onnx",   LaneModelType.UFLDV2_CULANE,   "V2"),
    "tusimple": ("models/tusimple_res18.onnx", LaneModelType.UFLDV2_TUSIMPLE, "V2"),
}

# ── LDW State Machine ─────────────────────────────────────────────────────────

class LDWSystem:
    """
    Implements the Lane Departure Warning gating logic from the ADAS report.

    Gates (all must pass before LDW fires):
      1. Lane Lock  — left+right ego lanes detected for ≥ LANE_LOCK_THRESH frames
      2. Speed Gate — ego_speed_kmh > MIN_SPEED_KMH
      3. Quality    — area_status True; if False for UNAVAILABLE_THRESH frames → "Lane Unavailable"
      4. Geometry   — vehicle path polygon is breached by a lane endpoint (pointPolygonTest)
      5. Cooldown   — at least LDW_COOLDOWN_SEC since last alert
    """

    def __init__(self):
        # Lane lock counters (increment on detect, decrement on miss, clamp 0..THRESH)
        self.left_lock_count  = 0
        self.right_lock_count = 0

        # Quality / unavailability tracking
        self.low_quality_streak = 0
        self.lane_available     = True

        # Departure direction
        self.departure_left  = False
        self.departure_right = False

        # Cooldown
        self.last_ldw_time = 0.0

        # State label
        self.state = "INITIALIZING"   # INITIALIZING | ARMED | DEPARTED | UNAVAILABLE

        # History for HUD graph (last 90 frames)
        self.quality_history = deque(maxlen=90)

    def update(self, lanes_points, lanes_status, area_status, ego_speed_kmh, car_path_poly):
        """
        Call once per frame. Returns (ldw_fired, departure_side).
        departure_side: "LEFT" | "RIGHT" | "BOTH" | None
        """
        self.departure_left  = False
        self.departure_right = False
        ldw_fired = False
        side = None

        # ── Quality / FR22 ────────────────────────────────────────────────────
        if area_status:
            self.low_quality_streak = 0
            self.lane_available = True
        else:
            self.low_quality_streak += 1
            if self.low_quality_streak >= UNAVAILABLE_THRESH:
                self.lane_available = False

        self.quality_history.append(1.0 if area_status else 0.0)

        # ── Lane lock counters ────────────────────────────────────────────────
        # lanes_status from V2: [left-side, left-ego, right-ego, right-side]
        # We care about ego lanes (index 1 = left-ego, index 2 = right-ego)
        left_ego_detected  = lanes_status[1] if len(lanes_status) > 1 else False
        right_ego_detected = lanes_status[2] if len(lanes_status) > 2 else False

        self.left_lock_count  = min(LANE_LOCK_THRESH,
                                    self.left_lock_count  + (1 if left_ego_detected  else -1))
        self.right_lock_count = min(LANE_LOCK_THRESH,
                                    self.right_lock_count + (1 if right_ego_detected else -1))
        self.left_lock_count  = max(0, self.left_lock_count)
        self.right_lock_count = max(0, self.right_lock_count)

        left_locked  = self.left_lock_count  >= LANE_LOCK_THRESH
        right_locked = self.right_lock_count >= LANE_LOCK_THRESH

        # ── Gate 1: Lane Unavailable ──────────────────────────────────────────
        if not self.lane_available:
            self.state = "UNAVAILABLE"
            return False, None

        # ── Gate 2: Speed ─────────────────────────────────────────────────────
        if ego_speed_kmh < MIN_SPEED_KMH:
            self.state = "INITIALIZING"
            return False, None

        # ── Gate 3: Lane Lock ─────────────────────────────────────────────────
        if not (left_locked or right_locked):
            self.state = "INITIALIZING"
            return False, None

        self.state = "ARMED"

        # ── Gate 4: Geometry (point-polygon test) ─────────────────────────────
        # Test bottom-most point of each ego lane against the car path polygon
        if left_locked and len(lanes_points) > 1 and len(lanes_points[1]) > 0:
            bottom_pt = _bottom_point(lanes_points[1])
            if bottom_pt and cv2.pointPolygonTest(car_path_poly, bottom_pt, False) >= 0:
                self.departure_left = True

        if right_locked and len(lanes_points) > 2 and len(lanes_points[2]) > 0:
            bottom_pt = _bottom_point(lanes_points[2])
            if bottom_pt and cv2.pointPolygonTest(car_path_poly, bottom_pt, False) >= 0:
                self.departure_right = True

        departed = self.departure_left or self.departure_right
        if not departed:
            return False, None

        # ── Gate 5: Cooldown ──────────────────────────────────────────────────
        now = time.time()
        if now - self.last_ldw_time < LDW_COOLDOWN_SEC:
            self.state = "DEPARTED"
            return False, None

        # ── Fire LDW ─────────────────────────────────────────────────────────
        self.last_ldw_time = now
        self.state = "DEPARTED"
        ldw_fired = True

        if self.departure_left and self.departure_right:
            side = "BOTH"
        elif self.departure_left:
            side = "LEFT"
        else:
            side = "RIGHT"

        return ldw_fired, side


def _bottom_point(lane_points):
    """Return the (x, y) point with the largest y (bottom of frame) in a lane."""
    if len(lane_points) == 0:
        return None
    pts = np.array(lane_points)
    idx = np.argmax(pts[:, 1])
    return (float(pts[idx, 0]), float(pts[idx, 1]))


# ── Drawing helpers ───────────────────────────────────────────────────────────

def draw_car_path(frame, poly):
    """Draw the vehicle projection polygon."""
    overlay = frame.copy()
    cv2.fillPoly(overlay, [poly.astype(np.int32)], COLOR_OVERLAY)
    cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)
    cv2.polylines(frame, [poly.astype(np.int32)], True, COLOR_OVERLAY, 2)


def draw_hud(frame, ldw_sys, ego_speed, fps, active_side, alert_flash):
    """Render the on-screen HUD overlay."""
    H, W = frame.shape[:2]
    panel_h = 130
    panel = np.zeros((panel_h, W, 3), dtype=np.uint8)
    panel[:] = (20, 20, 20)

    # ── State badge ───────────────────────────────────────────────────────────
    state = ldw_sys.state
    if state == "UNAVAILABLE":
        badge_color = COLOR_UNAVAIL
        label = "LANE UNAVAILABLE"
    elif state == "DEPARTED":
        badge_color = COLOR_CRITICAL
        label = f"LDW  {active_side or ''}".strip()
    elif state == "ARMED":
        badge_color = COLOR_SAFE
        label = "ARMED"
    else:
        badge_color = (80, 80, 80)
        label = "INITIALIZING"

    cv2.rectangle(panel, (10, 8), (260, 55), badge_color, -1)
    cv2.putText(panel, label, (18, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

    # ── Speed ─────────────────────────────────────────────────────────────────
    cv2.putText(panel, f"Speed: {ego_speed:.0f} km/h", (280, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                COLOR_WARNING if ego_speed < MIN_SPEED_KMH else (200, 200, 200), 2)

    # ── FPS ───────────────────────────────────────────────────────────────────
    cv2.putText(panel, f"FPS: {fps:.1f}", (480, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (160, 160, 160), 2)

    # ── Lane lock bars ────────────────────────────────────────────────────────
    def lock_bar(x, count, label_text):
        pct = count / LANE_LOCK_THRESH
        bar_w = 100
        filled = int(bar_w * pct)
        color = COLOR_SAFE if pct >= 1.0 else (0, 180, 255)
        cv2.rectangle(panel, (x, 65), (x + bar_w, 85), (60, 60, 60), -1)
        if filled > 0:
            cv2.rectangle(panel, (x, 65), (x + filled, 85), color, -1)
        cv2.rectangle(panel, (x, 65), (x + bar_w, 85), (100, 100, 100), 1)
        cv2.putText(panel, label_text, (x, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    lock_bar(10,  ldw_sys.left_lock_count,  "L-EGO LOCK")
    lock_bar(120, ldw_sys.right_lock_count, "R-EGO LOCK")

    # ── Quality sparkline ─────────────────────────────────────────────────────
    q_hist = list(ldw_sys.quality_history)
    if q_hist:
        spark_x, spark_y, spark_w, spark_h = 280, 62, 200, 35
        cv2.rectangle(panel, (spark_x, spark_y),
                      (spark_x + spark_w, spark_y + spark_h), (40, 40, 40), -1)
        n = len(q_hist)
        for i in range(1, n):
            x1 = spark_x + int((i - 1) * spark_w / max(n, 1))
            x2 = spark_x + int(i * spark_w / max(n, 1))
            y1 = spark_y + spark_h - int(q_hist[i - 1] * spark_h)
            y2 = spark_y + spark_h - int(q_hist[i] * spark_h)
            cv2.line(panel, (x1, y1), (x2, y2), COLOR_SAFE, 1)
        cv2.putText(panel, "Quality", (spark_x, spark_y + spark_h + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 140, 140), 1)

    # ── Unavailability streak bar ─────────────────────────────────────────────
    streak_pct = min(1.0, ldw_sys.low_quality_streak / UNAVAILABLE_THRESH)
    if streak_pct > 0:
        cv2.rectangle(panel, (500, 65), (600, 80), (60, 60, 60), -1)
        cv2.rectangle(panel, (500, 65),
                      (500 + int(100 * streak_pct), 80), COLOR_WARNING, -1)
        cv2.putText(panel, "Unavail streak", (500, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)

    # ── Paste panel onto frame ────────────────────────────────────────────────
    frame[:panel_h, :] = panel

    # ── Full-frame flash on departure ─────────────────────────────────────────
    if alert_flash and state == "DEPARTED":
        flash = frame.copy()
        flash[:] = (0, 0, 80)
        cv2.addWeighted(flash, 0.25, frame, 0.75, 0, frame)

        side_text = active_side or "LANE DEPARTURE"
        arrow = "<< " if active_side == "LEFT" else " >>" if active_side == "RIGHT" else "<<"
        msg = f"{arrow}  LDW: {side_text}  {arrow[::-1]}"
        text_size = cv2.getTextSize(msg, cv2.FONT_HERSHEY_DUPLEX, 1.8, 3)[0]
        tx = (W - text_size[0]) // 2
        ty = H // 2 + 40
        cv2.putText(frame, msg, (tx + 2, ty + 2),
                    cv2.FONT_HERSHEY_DUPLEX, 1.8, (0, 0, 0), 5)
        cv2.putText(frame, msg, (tx, ty),
                    cv2.FONT_HERSHEY_DUPLEX, 1.8, (0, 80, 255), 3)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="LDW — Lane Departure Warning System")
    parser.add_argument("--video",      type=str,   default="./temp/dashcam1.mp4")
    parser.add_argument("--model",      type=str,   default="culane", choices=MODEL_MAP.keys())
    parser.add_argument("--speed",      type=float, default=80.0,
                        help="Simulated ego speed in km/h (use your GPS/IMU value if available)")
    parser.add_argument("--no-display", action="store_true",
                        help="Skip live preview window")
    args = parser.parse_args()

    model_path, model_type, version = MODEL_MAP[args.model]

    # ── Load model ────────────────────────────────────────────────────────────
    print(f"Loading {version} model : {model_path}  ({model_type.name})")
    lane_detector = (UltrafastLaneDetectorV2 if version == "V2" else UltrafastLaneDetector)(
        model_path, model_type
    )

    # ── Open video ────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"ERROR: Could not open video '{args.video}'")
        exit(1)

    W      = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H      = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    # Build car path polygon in pixel coordinates
    car_path_poly = (CAR_PATH_POLY_FRAC * np.array([W, H])).astype(np.float32)

    output_path = args.video.rsplit(".", 1)[0] + "_ldw.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vout   = cv2.VideoWriter(output_path, fourcc, src_fps, (W, H))
    print(f"Writing output to: {output_path}")

    if not args.no_display:
        cv2.namedWindow("Lane Departure Warning", cv2.WINDOW_NORMAL)

    ldw = LDWSystem()

    fps           = 0.0
    frame_count   = 0
    perf_start    = time.time()

    # Alert flash persists for N frames after firing
    alert_flash       = False
    alert_flash_ttl   = 0
    ALERT_FLASH_FRAMES = int(src_fps * 1.5)   # 1.5 s

    active_side = None

    print("\nRunning — press Q to quit.\n")
    print(f"  Ego speed (simulated): {args.speed:.0f} km/h")
    print(f"  Speed gate:            > {MIN_SPEED_KMH:.0f} km/h")
    print(f"  Lane lock threshold:   {LANE_LOCK_THRESH} frames")
    print(f"  LDW cooldown:          {LDW_COOLDOWN_SEC:.1f} s\n")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # ── Run lane detection ────────────────────────────────────────────────
        lane_detector.DetectFrame(frame, adjust_lanes=True)
        info          = lane_detector.lane_info
        lanes_points  = info.lanes_points
        lanes_status  = list(info.lanes_status)
        area_status   = bool(info.area_status)

        # ── Draw detected lanes + filled area ─────────────────────────────────
        lane_detector.DrawDetectedOnFrame(frame)
        lane_detector.DrawAreaOnFrame(frame)

        # ── Draw vehicle path polygon ─────────────────────────────────────────
        draw_car_path(frame, car_path_poly)

        # ── Update LDW state machine ──────────────────────────────────────────
        fired, side = ldw.update(
            lanes_points, lanes_status, area_status,
            ego_speed_kmh=args.speed,
            car_path_poly=car_path_poly
        )

        if fired:
            active_side       = side
            alert_flash       = True
            alert_flash_ttl   = ALERT_FLASH_FRAMES
            print(f"  [LDW] Departure detected — {side}  (t={time.strftime('%H:%M:%S')})")

        if alert_flash_ttl > 0:
            alert_flash_ttl -= 1
        else:
            alert_flash = False
            if ldw.state != "DEPARTED":
                active_side = None

        # ── FPS counter ───────────────────────────────────────────────────────
        frame_count += 1
        if frame_count >= 30:
            fps         = frame_count / (time.time() - perf_start)
            frame_count = 0
            perf_start  = time.time()

        # ── HUD overlay ───────────────────────────────────────────────────────
        draw_hud(frame, ldw, args.speed, fps, active_side, alert_flash)

        vout.write(frame)

        if not args.no_display:
            cv2.imshow("Lane Departure Warning", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Stopped by user.")
                break

    vout.release()
    cap.release()
    cv2.destroyAllWindows()
    print(f"\nDone. Output saved to: {output_path}")


if __name__ == "__main__":
    main()
