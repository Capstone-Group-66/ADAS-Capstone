"""
imageDetection.py — Run lane detection on a single image.

Usage:
    python imageDetection.py --image path/to/image.jpg --model culane
    python imageDetection.py --image path/to/image.jpg --model tusimple
"""

import argparse
import cv2
from ufldDetector.utils import LaneModelType
from ufldDetector.ultrafastLaneDetector import UltrafastLaneDetector
from ufldDetector.ultrafastLaneDetectorV2 import UltrafastLaneDetectorV2

# Model map

MODEL_MAP = {
    "culane":   ("models/culane_res18.onnx",   LaneModelType.UFLDV2_CULANE,   "V2"),
    "tusimple": ("models/tusimple_res18.onnx", LaneModelType.UFLDV2_TUSIMPLE, "V2"),
}

# Args

parser = argparse.ArgumentParser(description="Lane detection on a single image.")
parser.add_argument("--image",  type=str, default="./temp/test.jpg")
parser.add_argument("--model",  type=str, default="culane", choices=MODEL_MAP.keys())
parser.add_argument("--output", type=str, default="output.jpg")
args = parser.parse_args()

model_path, model_type, version = MODEL_MAP[args.model]

# Load model

print(f"Loading {version} model : {model_path}  ({model_type.name})")
lane_detector = (UltrafastLaneDetectorV2 if version == "V2" else UltrafastLaneDetector)(
    model_path, model_type
)

# Load image

img = cv2.imread(args.image, cv2.IMREAD_COLOR)
if img is None:
    print(f"ERROR: Could not open image '{args.image}'")
    exit(1)

# Detect & draw

output_img = lane_detector.AutoDrawLanes(img)

cv2.imwrite(args.output, output_img)
print(f"Result saved to: {args.output}")

cv2.imshow("Detected lanes", output_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
