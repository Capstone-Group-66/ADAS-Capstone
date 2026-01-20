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
    # Determine model type by output shape
    # YOLOv8: [1, 84, 8400] (cx, cy, w, h, cls_scores...)
    # YOLOv5: [1, 25200, 85] (cx, cy, w, h, obj_conf, cls_scores...)
    
    output = outputs[0]
    
    boxes = []
    class_ids = []
    confidences = []
    
    x_factor = img_width / INPUT_WIDTH
    y_factor = img_height / INPUT_HEIGHT
    
    if output.shape[1] == 85: 
        # YOLOv5 Format [25200, 85]
        # Rows are detections
        rows = output.shape[0]
        
        # Vectorized filtering for YOLOv5
        # 4 box, 1 obj_conf, 80 cls_scores
        obj_conf = output[:, 4]
        cls_scores = output[:, 5:]
        
        # Filter by objectness first
        obj_mask = obj_conf >= SCORE_THRESHOLD
        
        filtered_obj_conf = obj_conf[obj_mask]
        filtered_cls_scores = cls_scores[obj_mask]
        filtered_output = output[obj_mask]
        
        # Now find best class for remaining
        class_ids = np.argmax(filtered_cls_scores, axis=1)
        cls_conf = filtered_cls_scores[np.arange(filtered_cls_scores.shape[0]), class_ids]
        
        # Final score = obj_conf * cls_conf
        final_scores = filtered_obj_conf * cls_conf
        
        # Filter by final score
        score_mask = final_scores >= SCORE_THRESHOLD
        
        filtered_output = filtered_output[score_mask]
        final_scores = final_scores[score_mask]
        class_ids = class_ids[score_mask]
        
        for i in range(filtered_output.shape[0]):
            row = filtered_output[i]
            cx, cy, w, h = row[0], row[1], row[2], row[3]
            
            left = int((cx - w/2) * x_factor)
            top = int((cy - h/2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            
            boxes.append([left, top, width, height])
            confidences.append(float(final_scores[i]))
            
    else:
        # YOLOv8 Format [84, 8400] -> Transpose to [8400, 84]
        output = output.transpose()
        
        scores = output[:, 4:]
        class_ids_all = np.argmax(scores, axis=1)
        max_scores = scores[np.arange(scores.shape[0]), class_ids_all]
        
        mask = max_scores >= SCORE_THRESHOLD
        
        filtered_output = output[mask]
        class_ids = class_ids_all[mask]
        confidences = max_scores[mask].tolist()
        
        for i in range(filtered_output.shape[0]):
            row = filtered_output[i]
            cx, cy, w, h = row[0], row[1], row[2], row[3]
            
            left = int((cx - w/2) * x_factor)
            top = int((cy - h/2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            
            boxes.append([left, top, width, height])
            
    # NMS
    indices = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD)
    
    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            i = int(i)
            box = boxes[i]
            results.append({
                "class_id": class_ids[i],
                "class_name": CLASS_NAMES[class_ids[i]],
                "confidence": confidences[i],
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
