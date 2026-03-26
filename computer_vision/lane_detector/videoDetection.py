"""
videoDetection.py — Run lane detection on a video file.

Usage:
    python videoDetection.py --video path/to/video.mp4 --model culane
    python videoDetection.py --video path/to/video.mp4 --model tusimple

Press Q to quit early.
Output is saved alongside the input file as <name>_out.mp4
"""

import argparse
import time
import cv2
from ufldDetector.utils import LaneModelType
from ufldDetector.ultrafastLaneDetector import UltrafastLaneDetector
from ufldDetector.ultrafastLaneDetectorV2 import UltrafastLaneDetectorV2

# Model map
# culane_res18.onnx / tusimple_res18.onnx are V2 models (4 outputs).
# V1 models have 1 output and use UFLD_CULANE / UFLD_TUSIMPLE types.

MODEL_MAP = {
    "culane":   ("models/culane_res18.onnx",   LaneModelType.UFLDV2_CULANE,   "V2"),
    "tusimple": ("models/tusimple_res18.onnx", LaneModelType.UFLDV2_TUSIMPLE, "V2"),
}

# Args

parser = argparse.ArgumentParser(description="Lane detection on a video.")
parser.add_argument("--video",      type=str, default="./data/dashcam6.mp4")
parser.add_argument("--model",      type=str, default="culane", choices=MODEL_MAP.keys())
parser.add_argument("--no-display", action="store_true",
                    help="Skip the live preview window (useful on headless servers)")
args = parser.parse_args()

model_path, model_type, version = MODEL_MAP[args.model]

# Load model

print(f"Loading {version} model : {model_path}  ({model_type.name})")
lane_detector = (UltrafastLaneDetectorV2 if version == "V2" else UltrafastLaneDetector)(
    model_path, model_type
)

# Open video

cap = cv2.VideoCapture(args.video)
if not cap.isOpened():
    print(f"ERROR: Could not open video '{args.video}'")
    exit(1)

width      = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
source_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

output_path = args.video.rsplit(".", 1)[0] + "_out.mp4"
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
vout   = cv2.VideoWriter(output_path, fourcc, source_fps, (width, height))
print(f"Writing output to: {output_path}")

if not args.no_display:
    cv2.namedWindow("Detected lanes", cv2.WINDOW_NORMAL)

# Main loop

fps         = 0.0
frame_count = 0
start       = time.time()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    output_img = lane_detector.AutoDrawLanes(frame)

    frame_count += 1
    if frame_count >= 30:
        fps         = frame_count / (time.time() - start)
        frame_count = 0
        start       = time.time()

    cv2.putText(output_img, f"FPS: {fps:.1f}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    vout.write(output_img)

    if not args.no_display:
        cv2.imshow("Detected lanes", output_img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Stopped by user.")
            break

vout.release()
cap.release()
cv2.destroyAllWindows()
print("Done.")
