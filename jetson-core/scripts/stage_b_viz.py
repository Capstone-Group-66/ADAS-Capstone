#!/usr/bin/env python3
"""
Stage B Visualization Tool
Shows live camera feed with:
- Undistortion applied
- YOLOv8 object detection overlay (if model available)
- Performance metrics

Press 'U' to toggle undistortion, 'Q' to quit.
"""
import cv2
import numpy as np
import sys
import os
import time

def load_calibration(yaml_path):
    """Load calibration from OpenCV FileStorage YAML"""
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        return None, None, 0, 0, 0
    
    mtx = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("distortion_coefficients").mat()
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    rms = fs.getNode("rms_error").real()
    fs.release()
    
    return mtx, dist, w, h, rms

def load_yolo_model(model_path):
    """Load YOLOv8 ONNX model"""
    if not os.path.exists(model_path):
        print(f"[Stage B Viz] Model not found at {model_path}")
        return None
    
    try:
        net = cv2.dnn.readNetFromONNX(model_path)
        # Use CPU backend for maximum compatibility
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        print("[Stage B Viz] Using CPU backend")
        return net
    except Exception as e:
        print(f"[Stage B Viz] Failed to load model: {e}")
        return None

def run_inference(net, frame, conf_threshold=0.25, nms_threshold=0.45):
    """Run YOLOv8 inference and return detections"""
    input_size = (640, 640)
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, input_size, swapRB=True, crop=False)
    net.setInput(blob)
    
    start = time.time()
    output = net.forward()
    inference_time = (time.time() - start) * 1000  # ms
    
    # YOLOv8 output: [1, 84, 8400] -> transpose to [8400, 84]
    output = output[0].T
    
    # Scale factors
    h, w = frame.shape[:2]
    x_scale = w / input_size[0]
    y_scale = h / input_size[1]
    
    boxes = []
    confidences = []
    class_ids = []
    
    for row in output:
        cx, cy, bw, bh = row[:4]
        scores = row[4:]
        max_score = np.max(scores)
        class_id = np.argmax(scores)
        
        if max_score >= conf_threshold:
            x = int((cx - bw/2) * x_scale)
            y = int((cy - bh/2) * y_scale)
            width = int(bw * x_scale)
            height = int(bh * y_scale)
            
            boxes.append([x, y, width, height])
            confidences.append(float(max_score))
            class_ids.append(int(class_id))
    
    # NMS
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    
    detections = []
    for i in indices:
        idx = i if isinstance(i, int) else i[0]
        detections.append({
            'box': boxes[idx],
            'confidence': confidences[idx],
            'class_id': class_ids[idx]
        })
    
    return detections, inference_time

# COCO class names
CLASS_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

def draw_detections(frame, detections):
    """Draw detection boxes on frame"""
    for det in detections:
        x, y, w, h = det['box']
        conf = det['confidence']
        class_id = det['class_id']
        
        # Color based on class (cars green, people blue, others red)
        if class_id == 2:  # car
            color = (0, 255, 0)
        elif class_id == 0:  # person
            color = (255, 0, 0)
        else:
            color = (0, 0, 255)
        
        # Draw box
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
        
        # Draw label
        label = f"{CLASS_NAMES[class_id]}: {conf:.2f}"
        (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x, y-label_h-5), (x+label_w, y), color, -1)
        cv2.putText(frame, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    
    return frame

def main():
    # Get paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    jetson_core_dir = os.path.dirname(script_dir)
    
    calib_path = os.path.join(jetson_core_dir, "config", "calibration", "FrontCam_calibration.yaml")
    model_path = os.path.join(jetson_core_dir, "models", "yolov8n.onnx")
    hw_map_path = os.path.join(jetson_core_dir, "config", "hardware_map.json")
    
    # Load calibration
    mtx, dist, cal_w, cal_h, rms = load_calibration(calib_path)
    if mtx is not None:
        print(f"[Stage B Viz] Calibration loaded (RMS: {rms:.4f})")
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (cal_w, cal_h), 1, (cal_w, cal_h))
        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, new_mtx, (cal_w, cal_h), cv2.CV_32FC1)
        calibrated = True
    else:
        print("[Stage B Viz] No calibration found - running without undistortion")
        calibrated = False
    
    # Load YOLO model
    net = load_yolo_model(model_path)
    detection_enabled = net is not None
    
    # Get camera ID
    cam_id = 1
    if os.path.exists(hw_map_path):
        import json
        with open(hw_map_path, 'r') as f:
            hw = json.load(f)
            if "mappings" in hw and "FrontCam" in hw["mappings"]:
                cam_id = int(hw["mappings"]["FrontCam"])
    
    # Open camera
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print(f"[Stage B Viz] Cannot open camera {cam_id}")
        sys.exit(1)
    
    if calibrated:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cal_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cal_h)
    
    # State
    undistort_enabled = calibrated
    detect_enabled = detection_enabled
    
    print("\n" + "="*60)
    print("STAGE B VISUALIZATION")
    print("="*60)
    print("  Press 'U' to toggle undistortion")
    print("  Press 'D' to toggle detection")
    print("  Press 'Q' to quit")
    print("="*60 + "\n")
    
    fps_history = []
    
    while True:
        frame_start = time.time()
        
        ret, frame = cap.read()
        if not ret:
            break
        
        # Stage B Step 1: Undistortion
        if undistort_enabled and calibrated:
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        
        # Stage B Step 2: Detection
        inference_time = 0
        detections = []
        if detect_enabled and net is not None:
            detections, inference_time = run_inference(net, frame)
            frame = draw_detections(frame, detections)
        
        # Calculate FPS
        frame_time = time.time() - frame_start
        fps = 1.0 / max(frame_time, 0.001)
        fps_history.append(fps)
        if len(fps_history) > 30:
            fps_history.pop(0)
        avg_fps = sum(fps_history) / len(fps_history)
        
        # Draw status overlay
        status_lines = [
            f"FPS: {avg_fps:.1f}",
            f"Undistort: {'ON' if undistort_enabled else 'OFF'}",
            f"Detection: {'ON' if detect_enabled else 'OFF'}",
            f"Objects: {len(detections)}",
        ]
        if inference_time > 0:
            status_lines.append(f"Inference: {inference_time:.1f}ms")
        
        y_offset = 30
        for line in status_lines:
            cv2.putText(frame, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.6, (0, 255, 0), 2)
            y_offset += 25
        
        cv2.imshow("Stage B Visualization", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('u'):
            undistort_enabled = not undistort_enabled
            print(f"Undistortion: {'ON' if undistort_enabled else 'OFF'}")
        elif key == ord('d'):
            detect_enabled = not detect_enabled
            print(f"Detection: {'ON' if detect_enabled else 'OFF'}")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
