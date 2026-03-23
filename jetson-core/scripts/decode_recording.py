#!/usr/bin/env python3
"""
decode_recording.py — ADAS Recording decoder
Works dynamically based on devices plugged in during the run.
Decodes .adasrec files and dumps data into a folder named after the recording.
"""

import struct
import sys
import os
import argparse
import csv
import subprocess
from collections import defaultdict
from datetime import datetime, timezone

# ── File format constants ──────────────────────────────────────────────────────
MAGIC = b'AREC'
FILE_HEADER_SIZE = 32   # AdasRecFileHeader
EVENT_HEADER_SIZE = 13  # RecEventHeader

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
    0x40: 'FrontDetBatch',
    0x50: 'RCWState',
}

CAMERA_EVENTS = {0x01, 0x02, 0x03, 0x04}
RADAR_EVENTS  = {0x10, 0x11, 0x12}
IMU_EVENTS    = {0x20}

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
    if version not in (1, 3):
        raise ValueError(f"Unsupported version: {version}")
    return version, start_ns, sensor_mask

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
        return None, None, None, None
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

def decode_front_det_batch(payload):
    if len(payload) < 16:
        return None
    num_dets, _reserved, inference_time_us = struct.unpack_from('<IIQ', payload, 0)
    offset = 16
    dets = []
    per_det = 72
    for _ in range(num_dets):
        if offset + per_det > len(payload):
            break
        vals = struct.unpack_from('<6fifQ32s', payload, offset)
        sign_label = vals[9].split(b'\x00', 1)[0].decode('utf-8', errors='ignore')
        dets.append({
            'x': vals[0],
            'y': vals[1],
            'w': vals[2],
            'h': vals[3],
            'centroid_x': vals[4],
            'centroid_y': vals[5],
            'cls': vals[6],
            'score': vals[7],
            'object_id': vals[8],
            'sign_label': sign_label,
        })
        offset += per_det
    return {
        'num_detections': num_dets,
        'inference_time_us': inference_time_us,
        'detections': dets,
    }

def decode_rcw_payload(payload):
    if len(payload) < 2:
        return None
    alert, status = struct.unpack_from('<BB', payload, 0)
    return {'alert': alert, 'status': status}

# ── Video Generation ─────────────────────────────────────────────────────────
def compile_video(image_folder, output_path, fps=20.0):
    try:
        import cv2
        import glob
    except ImportError:
        print("[WARNING] OpenCV not installed. Skipping video compilation.")
        return

    search_path = os.path.join(image_folder, "*.jpg")
    images = glob.glob(search_path)
    
    if not images:
        return

    # Sort strictly by the timestamp in the filename
    images.sort(key=lambda x: int(os.path.basename(x).replace('.jpg', '')))

    first_frame = cv2.imread(images[0])
    if first_frame is None:
        return
        
    height, width, layers = first_frame.shape
    size = (width, height)

    print(f"  -> Compiling {len(images)} frames into {os.path.basename(output_path)} at {fps} FPS...")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, size)

    for i, img_path in enumerate(images):
        frame = cv2.imread(img_path)
        out.write(frame)

    out.release()


# ── Decoding ───────────────────────────────────────────────────────────────────
def decode_recording(path, fps=20.0):
    print(f"Decoding {path}...")
    base_dir = os.path.dirname(os.path.abspath(path))
    file_name = os.path.basename(path)
    folder_name = os.path.splitext(file_name)[0]
    
    # Create the base output directory
    out_dir = os.path.join(base_dir, folder_name)
    os.makedirs(out_dir, exist_ok=True)
    print(f"Output directory: {out_dir}")

    csv_files = {}
    csv_writers = {}
    
    def get_csv_writer(name, header):
        if name not in csv_writers:
            f = open(os.path.join(out_dir, f"{name}.csv"), 'w', newline='')
            csv_files[name] = f
            writer = csv.writer(f)
            writer.writerow(header)
            csv_writers[name] = writer
        return csv_writers[name]

    counts = defaultdict(int)

    with open(path, 'rb') as f:
        try:
            version, start_ns, sensor_mask = parse_file_header(f)
        except Exception as e:
            print(f"[ERROR] Failed to parse file header: {e}")
            return
        print(f"Recording version: v{version}")
            
        while True:
            try:
                hdr = parse_event_header(f)
                if hdr is None:
                    break
                
                ts_ns, event_type, payload_size = hdr
                payload = f.read(payload_size)
                
                if len(payload) < payload_size:
                    print(f"[WARNING] Truncated payload for event type {event_type}. Stopping.")
                    break
                
                name = EVENT_TYPES.get(event_type, f'UNKNOWN(0x{event_type:02X})')
                counts[name] += 1
                
                if event_type in CAMERA_EVENTS:
                    mount, width, height, jpeg = decode_camera_payload(payload)
                    if jpeg:
                        cam_dir = os.path.join(out_dir, name)
                        # We only want to run os.makedirs once per camera, but exist_ok=True is fast enough
                        os.makedirs(cam_dir, exist_ok=True)
                        jpeg_path = os.path.join(cam_dir, f"{ts_ns}.jpg")
                        with open(jpeg_path, 'wb') as jf:
                            jf.write(jpeg)
                            
                elif event_type in RADAR_EVENTS:
                    targets = decode_radar_payload(payload)
                    if targets:
                        writer = get_csv_writer(name, ['timestamp_ns', 'range_m', 'azimuth_rad', 'vel_mps', 'rcs_db'])
                        for t in targets:
                            writer.writerow([ts_ns, t['range_m'], t['azimuth_rad'], t['vel_mps'], t['rcs_db']])
                            
                elif event_type in IMU_EVENTS:
                    imu_data = decode_imu_payload(payload)
                    if imu_data:
                        writer = get_csv_writer(name, [
                            'timestamp_ns', 'accel_x', 'accel_y', 'accel_z',
                            'gyro_x', 'gyro_y', 'gyro_z',
                            'quat_w', 'quat_x', 'quat_y', 'quat_z'
                        ])
                        writer.writerow([
                            ts_ns,
                            imu_data['accel'][0], imu_data['accel'][1], imu_data['accel'][2],
                            imu_data['gyro'][0], imu_data['gyro'][1], imu_data['gyro'][2],
                            imu_data['quat'][0], imu_data['quat'][1], imu_data['quat'][2], imu_data['quat'][3]
                        ])
                elif event_type == 0x40:
                    batch = decode_front_det_batch(payload)
                    if batch:
                        writer = get_csv_writer(
                            name,
                            ['timestamp_ns', 'inference_time_us', 'det_index', 'cls',
                             'score', 'object_id', 'sign_label', 'x', 'y', 'w', 'h',
                             'centroid_x', 'centroid_y']
                        )
                        if not batch['detections']:
                            writer.writerow([ts_ns, batch['inference_time_us'], -1,
                                             '', '', '', '', '', '', '', '', '', ''])
                        for idx, det in enumerate(batch['detections']):
                            writer.writerow([
                                ts_ns,
                                batch['inference_time_us'],
                                idx,
                                det['cls'],
                                det['score'],
                                det['object_id'],
                                det['sign_label'],
                                det['x'],
                                det['y'],
                                det['w'],
                                det['h'],
                                det['centroid_x'],
                                det['centroid_y'],
                            ])
                elif event_type == 0x50:
                    rcw = decode_rcw_payload(payload)
                    if rcw:
                        writer = get_csv_writer(name, ['timestamp_ns', 'alert', 'status'])
                        writer.writerow([ts_ns, rcw['alert'], rcw['status']])
                        
            except Exception as e:
                print(f"[WARNING] Parse error during iteration: {e}")
                break

    # Close all open CSV files
    for key, f in csv_files.items():
        f.close()

    print("\nDecoding complete. Events processed:")
    for k, v in sorted(counts.items()):
        print(f"  {k}: {v}")

    print("\nCompiling MP4 Videos...")
    for event_type_name in EVENT_TYPES.values():
        if event_type_name in counts and event_type_name in [EVENT_TYPES[k] for k in CAMERA_EVENTS]:
            cam_dir = os.path.join(out_dir, event_type_name)
            mp4_path = os.path.join(out_dir, f"{event_type_name}.mp4")
            compile_video(cam_dir, mp4_path, fps=fps)

    print("\n[SUCCESS] Fully Decoded Pipeline Complete.")

def main():
    parser = argparse.ArgumentParser(description='.adasrec decoder tool')
    parser.add_argument('file', help='Path to .adasrec file')
    parser.add_argument('--fps', type=float, default=20.0, help='Framerate for MP4 compilation (default: 20 based on camera ingest node configuration)')
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"[ERROR] File not found: {args.file}")
        sys.exit(1)

    decode_recording(args.file, fps=args.fps)

if __name__ == '__main__':
    main()
