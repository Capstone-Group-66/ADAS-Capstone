#!/usr/bin/env python3
"""
Debug YOLOv8 ONNX Model Locally (Windows/Dev Machine)

This script replicates the C++ ObjectDetector logic using Python and OpenCV.
It allows you to test the exact same ONNX model and post-processing logic
that is running on the Jetson, but on your local webcam.

Usage:
    python scripts/debug_yolo_onnx_local.py --model models/yolov8n_cv41.onnx
"""

import cv2
import numpy as np
import argparse
import sys

# Constants
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.25 # Default low threshold for debugging
NMS_THRESHOLD = 0.45
CONFIDENCE_THRESHOLD = 0.45

# COCO Class Names (80 classes)
CLASS_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
]

def postprocess(outputs, img_width, img_height):
    # YOLOv8 output shape is [1, 84, 8400]
    output = outputs[0].transpose() # [8400, 84]
    
    # Scale factors
    x_factor = img_width / INPUT_WIDTH
    y_factor = img_height / INPUT_HEIGHT
    
    # Vectorized filtering
    # Get max score and class ID for each row
    # row[:, 4:] contains the 80 class scores
    scores = output[:, 4:]
    class_ids = np.argmax(scores, axis=1)
    max_scores = scores[np.arange(scores.shape[0]), class_ids]
    
    # Filter by confidence
    mask = max_scores >= SCORE_THRESHOLD
    
    filtered_output = output[mask]
    filtered_class_ids = class_ids[mask]
    filtered_confidences = max_scores[mask]
    
    boxes = []
    
    # Iterate only over high-confidence detections
    for i in range(filtered_output.shape[0]):
        row = filtered_output[i]
        
        # YOLOv8 returns cx, cy, w, h
        cx, cy, w, h = row[0], row[1], row[2], row[3]
        
        # Convert to top-left x, y
        left = int((cx - w/2) * x_factor)
        top = int((cy - h/2) * y_factor)
        width = int(w * x_factor)
        height = int(h * y_factor)
        
        boxes.append([left, top, width, height])
            
    # NMS
    # OpenCV NMS requires a list of ints for class_ids and floats for confidences
    # But NMSBoxes expects list of Rect (x,y,w,h) and list of scores. 
    # It doesn't use class_ids, so we do it per-class if we want strict per-class NMS, 
    # but standard YOLO often does global NMS or class-aware.
    # cv2.dnn.NMSBoxes expects boxes as list of [x, y, w, h]
    
    indices = cv2.dnn.NMSBoxes(boxes, filtered_confidences.tolist(), SCORE_THRESHOLD, NMS_THRESHOLD)
    
    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            box = boxes[i]
            results.append({
                "class_id": filtered_class_ids[i],
                "class_name": CLASS_NAMES[filtered_class_ids[i]],
                "confidence": filtered_confidences[i],
                "box": box
            })
        
    return results

def load_calibration(yaml_path):
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f"Error: Cannot open {yaml_path}")
        return None
        
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    width = int(fs.getNode("image_width").real())
    height = int(fs.getNode("image_height").real())
    
    fs.release()
    
    return {
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "width": width,
        "height": height
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default="models/yolov8n.onnx", help="Path to ONNX model")
    parser.add_argument("--calib", type=str, default="config/calibration/FrontCam_calibration.yaml", help="Path to calibration YAML")
    parser.add_argument("--cam", type=int, default=0, help="Camera index")
    args = parser.parse_args()
    
    print(f"Loading model: {args.model}")
    try:
        net = cv2.dnn.readNetFromONNX(args.model)
    except cv2.error as e:
        print(f"Error loading model: {e}")
        return

    # Load Calibration
    calib_data = load_calibration(args.calib)
    map1, map2 = None, None
    roi = None
    
    if calib_data:
        print(f"Loaded calibration from {args.calib}")
        h, w = calib_data["height"], calib_data["width"]
        
        # Get optimal new camera matrix (matches C++ logic)
        # alpha=1.0 keeps all pixels
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            calib_data["camera_matrix"],
            calib_data["dist_coeffs"],
            (w, h),
            1.0,
            (w, h)
        )
        
        # Precompute maps
        map1, map2 = cv2.initUndistortRectifyMap(
            calib_data["camera_matrix"],
            calib_data["dist_coeffs"],
            None,
            new_camera_matrix,
            (w, h),
            cv2.CV_32FC1
        )
        print(f"Undistortion enabled. ROI: {roi}")
    else:
        print("Warning: Calibration not found, running raw.")

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"Cannot open camera {args.cam}")
        return
    
    # Force camera to expected resolution if possible
    if calib_data:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib_data["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib_data["height"])
        
    cv2.namedWindow("YOLOv8 Debug (Undistorted)", cv2.WINDOW_NORMAL)
    
    print("Starting inference loop. Press 'q' to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Apply Undistortion & Cropping matched to C++
        processed_frame = frame
        if map1 is not None and map2 is not None:
            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            
            # Crop to ROI (simulate crop_to_roi=true)
            if roi is not None:
                x, y, w, h = roi
                processed_frame = undistorted[y:y+h, x:x+w]
            else:
                processed_frame = undistorted

        img_height, img_width = processed_frame.shape[:2]
        
        # Preprocess
        blob = cv2.dnn.blobFromImage(processed_frame, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        net.setInput(blob)
        
        # Inference
        outputs = net.forward()
        
        # Postprocess
        detections = postprocess(outputs, img_width, img_height)
        
        # Draw on processed frame
        for det in detections:
            box = det["box"]
            x, y, w, h = box
            
            color = (0, 255, 0)
            if det["class_name"] == "person":
                color = (0, 0, 255) 
            
            cv2.rectangle(processed_frame, (x, y), (x+w, y+h), color, 2)
            
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.putText(processed_frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
        # Info
        cv2.putText(processed_frame, f"Detections: {len(detections)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        cv2.imshow("YOLOv8 Debug (Undistorted)", processed_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
