#!/usr/bin/env python3
"""
view_recording.py — Cross-platform .adasrec file inspector and viewer
Works on Windows/macOS/Linux. Requires: pip install opencv-python numpy

Usage:
  python view_recording.py run_20260228_110404.adasrec          # inspect + play
  python view_recording.py run_20260228_110404.adasrec --stats  # stats only
  python view_recording.py run_20260228_110404.adasrec --mount FrontCam  # one cam
"""

import struct
import sys
import os
import argparse
from collections import defaultdict
from datetime import datetime, timezone

# ── File format constants (must match Recorder.hpp) ───────────────────────────
MAGIC = b'AREC'
FILE_HEADER_SIZE = 32   # AdasRecFileHeader
EVENT_HEADER_SIZE = 13  # RecEventHeader (packed: uint64 + uint8 + uint32)

EVENT_TYPES = {
    0x01: 'CameraFront',
    0x02: 'CameraSideL',
    0x03: 'CameraSideR',
    0x04: 'CameraRear',
    0x10: 'RadarFront',
    0x11: 'RadarRearL',
    0x12: 'RadarRearR',
    0x20: 'IMU',
    0x30: 'GPS',
}

CAMERA_EVENTS = {0x01, 0x02, 0x03, 0x04}
RADAR_EVENTS  = {0x10, 0x11, 0x12}

MOUNT_FILTER = {
    'FrontCam':  0x01,
    'SideCamL':  0x02,
    'SideCamR':  0x03,
    'RearCam':   0x04,
    'FrontRadar':0x10,
    'IMU':       0x20,
}

# ── Parser ─────────────────────────────────────────────────────────────────────
def parse_file_header(f):
    raw = f.read(FILE_HEADER_SIZE)
    if len(raw) < FILE_HEADER_SIZE:
        raise ValueError("File too short to contain header")
    magic      = raw[0:4]
    version    = struct.unpack_from('<H', raw, 4)[0]
    start_ns   = struct.unpack_from('<Q', raw, 8)[0]
    sensor_mask= struct.unpack_from('<H', raw, 16)[0]
    if magic != MAGIC:
        raise ValueError(f"Bad magic: {magic!r} (expected b'AREC')")
    if version != 1:
        raise ValueError(f"Unsupported version: {version}")
    return start_ns, sensor_mask

def parse_event_header(f):
    raw = f.read(EVENT_HEADER_SIZE)
    if len(raw) == 0:
        return None  # EOF
    if len(raw) < EVENT_HEADER_SIZE:
        raise ValueError(f"Truncated event header: got {len(raw)}B")
    ts_ns,       = struct.unpack_from('<Q', raw, 0)
    event_type,  = struct.unpack_from('<B', raw, 8)
    payload_size,= struct.unpack_from('<I', raw, 9)
    return ts_ns, event_type, payload_size

def decode_camera_payload(payload):
    if len(payload) < 5:
        return None, None, None
    mount  = payload[0]
    width  = struct.unpack_from('<H', payload, 1)[0]
    height = struct.unpack_from('<H', payload, 3)[0]
    jpeg   = payload[5:]
    return mount, width, height, bytes(jpeg)

def decode_radar_payload(payload):
    if len(payload) < 1:
        return []
    n = payload[0]
    targets = []
    offset = 1
    per = 7 * 4  # 7 floats
    for _ in range(n):
        if offset + per > len(payload):
            break
        vals = struct.unpack_from('<7f', payload, offset)
        targets.append({
            'range_m':     vals[0],
            'azimuth_rad': vals[1],
            'vel_mps':     vals[2],
            'rcs_db':      vals[3],
        })
        offset += per
    return targets

def decode_imu_payload(payload):
    if len(payload) < 57:
        return None
    accel = struct.unpack_from('<3f', payload, 0)
    gyro  = struct.unpack_from('<3f', payload, 12)
    quat  = struct.unpack_from('<4f', payload, 36)
    return {'accel': accel, 'gyro': gyro, 'quat': quat}

# ── Stats pass ─────────────────────────────────────────────────────────────────
def print_stats(path):
    counts    = defaultdict(int)
    first_ts  = None
    last_ts   = None
    errors    = 0

    with open(path, 'rb') as f:
        start_ns, sensor_mask = parse_file_header(f)
        print(f"  Magic:        AREC [OK]")
        dt = datetime.fromtimestamp(start_ns / 1e9, tz=timezone.utc)
        print(f"  Recorded at:  {dt.strftime('%Y-%m-%d %H:%M:%S UTC')}")
        print(f"  Sensor mask:  0x{sensor_mask:04X}")
        print()

        while True:
            try:
                hdr = parse_event_header(f)
                if hdr is None:
                    break
                ts_ns, event_type, payload_size = hdr
                payload = f.read(payload_size)
                if len(payload) < payload_size:
                    errors += 1
                    break

                name = EVENT_TYPES.get(event_type, f'UNKNOWN(0x{event_type:02X})')
                counts[name] += 1

                if first_ts is None:
                    first_ts = ts_ns
                last_ts = ts_ns

            except Exception as e:
                errors += 1
                print(f"  [!] Parse error: {e}")
                break

    print("  Event counts:")
    total = 0
    for name, count in sorted(counts.items()):
        print(f"    {name:<20} {count:>6} events")
        total += count
    print(f"    {'TOTAL':<20} {total:>6}")
    print()

    if first_ts and last_ts:
        duration_s = (last_ts - first_ts) / 1e9
        print(f"  Duration:     {duration_s:.2f}s")
        cam_total = sum(counts.get(n, 0) for n in ['CameraFront','CameraSideL','CameraSideR','CameraRear'])
        if duration_s > 0 and cam_total > 0:
            print(f"  Approx FPS:   {cam_total / duration_s:.1f} frames/sec (all cameras combined)")
    if errors:
        print(f"  [!] {errors} parse error(s) — file may be truncated")
    else:
        print(f"  File integrity: OK [PASS]")

    file_size = os.path.getsize(path)
    print(f"  File size:    {file_size / (1024*1024):.1f} MB")

# ── Playback ───────────────────────────────────────────────────────────────────
def play(path, mount_filter=None):
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("[ERROR] opencv-python not installed. Run: pip install opencv-python")
        sys.exit(1)

    filter_type = MOUNT_FILTER.get(mount_filter) if mount_filter else None

    events = []
    with open(path, 'rb') as f:
        parse_file_header(f)
        while True:
            hdr = parse_event_header(f)
            if hdr is None:
                break
            ts_ns, event_type, payload_size = hdr
            payload = f.read(payload_size)
            if len(payload) < payload_size:
                break
            if event_type in CAMERA_EVENTS:
                if filter_type is None or event_type == filter_type:
                    events.append((ts_ns, event_type, payload))

    if not events:
        print("[Viewer] No camera events found for the selected mount.")
        return

    print(f"[Viewer] Loaded {len(events)} camera frames. Press Q to quit, SPACE to pause.")

    first_ts = events[0][0]
    paused = False

    windows = {}

    import time
    wall_start = time.time()
    replay_start_ns = first_ts

    for ts_ns, event_type, payload in events:
        mount, width, height, jpeg = decode_camera_payload(payload)
        if not jpeg:
            continue

        frame_arr = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(frame_arr, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        name = EVENT_TYPES.get(event_type, 'Camera')
        ts_rel = (ts_ns - first_ts) / 1e9

        # Overlay info
        cv2.putText(frame, f"{name}  t={ts_rel:.2f}s  {width}x{height}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.putText(frame, "REPLAY — press Q to quit, SPACE to pause",
                    (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)

        win = f"ADASREC: {name}"
        cv2.imshow(win, frame)

        # Timing: sleep to match original timestamps (realtime replay)
        target_wall = wall_start + (ts_ns - replay_start_ns) / 1e9
        sleep_s = target_wall - time.time()
        if sleep_s > 0:
            key = cv2.waitKey(max(1, int(sleep_s * 1000)))
        else:
            key = cv2.waitKey(1)

        if key == ord('q') or key == 27:
            break
        if key == ord(' '):
            paused = not paused
            while paused:
                k = cv2.waitKey(100)
                if k == ord(' ') or k == ord('q') or k == 27:
                    if k != ord(' '):
                        paused = False
                        break
                    paused = False

    cv2.destroyAllWindows()
    print("[Viewer] Done.")

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='.adasrec inspector and viewer')
    parser.add_argument('file', help='Path to .adasrec file')
    parser.add_argument('--stats', action='store_true', help='Print stats only, no playback')
    parser.add_argument('--mount', choices=list(MOUNT_FILTER.keys()),
                        help='Only show frames from this mount (default: all cameras)')
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"[ERROR] File not found: {args.file}")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  ADASREC Inspector — {os.path.basename(args.file)}")
    print(f"{'='*60}")
    print_stats(args.file)
    print(f"{'='*60}\n")

    if not args.stats:
        ans = input("Play back camera frames? [Y/n]: ").strip().lower()
        if ans in ('', 'y', 'yes'):
            play(args.file, mount_filter=args.mount)

if __name__ == '__main__':
    main()
