#!/usr/bin/env python3
"""
Calibration Visualization Tool
Shows live camera feed with/without undistortion applied.
Press 'U' to toggle undistortion, 'Q' to quit.
"""
import cv2
import numpy as np
import sys
import os

def load_calibration(yaml_path):
    """Load calibration from OpenCV FileStorage YAML"""
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f"ERROR: Cannot open {yaml_path}")
        return None, None, None, None
    
    mtx = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("distortion_coefficients").mat()
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    rms = fs.getNode("rms_error").real()
    
    fs.release()
    
    print(f"Loaded calibration from: {yaml_path}")
    print(f"  Image size: {w}x{h}")
    print(f"  RMS error: {rms:.4f}")
    print(f"  Camera matrix:\n{mtx}")
    print(f"  Distortion coefficients: {dist.flatten()}")
    
    return mtx, dist, w, h

def main():
    # Get the script's directory (jetson-core/scripts)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    jetson_core_dir = os.path.dirname(script_dir)  # Go up one level to jetson-core
    
    # Find calibration file
    calib_path = os.path.join(jetson_core_dir, "config", "calibration", "FrontCam_calibration.yaml")
    if not os.path.exists(calib_path):
        print(f"ERROR: Calibration file not found at {calib_path}")
        print("Run device_wizard --calibrate first!")
        sys.exit(1)
    
    # Load calibration
    mtx, dist, cal_w, cal_h = load_calibration(calib_path)
    if mtx is None:
        sys.exit(1)
    
    # Get camera ID from hardware_map or default to 1
    cam_id = 1
    hw_map_path = os.path.join(jetson_core_dir, "config", "hardware_map.json")
    if os.path.exists(hw_map_path):
        import json
        with open(hw_map_path, 'r') as f:
            hw = json.load(f)
            if "mappings" in hw and "FrontCam" in hw["mappings"]:
                cam_id = int(hw["mappings"]["FrontCam"])
                print(f"Using camera ID {cam_id} from hardware_map.json")
    
    # Open camera
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {cam_id}")
        sys.exit(1)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cal_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cal_h)
    
    # Compute undistort maps for efficiency
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (cal_w, cal_h), 1, (cal_w, cal_h))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, new_mtx, (cal_w, cal_h), cv2.CV_32FC1)
    
    # Load RMS for display
    fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
    rms_error = fs.getNode("rms_error").real()
    fs.release()
    
    # Modes: 0=Original, 1=Undistorted, 2=Undistorted+Cropped
    mode = 0
    mode_names = ["ORIGINAL", "UNDISTORTED (full)", "UNDISTORTED + CROPPED"]
    
    print("\n" + "="*60)
    print("CALIBRATION VISUALIZATION")
    print("="*60)
    print("  Press 'U' to cycle modes: Original -> Undistorted -> Cropped")
    print("  Press 'Q' to quit")
    print(f"  ROI for cropping: x={roi[0]}, y={roi[1]}, w={roi[2]}, h={roi[3]}")
    print("="*60 + "\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break
        
        if mode == 0:
            # Original
            display = frame.copy()
            color = (0, 0, 255)
        elif mode == 1:
            # Undistorted (full frame with artifacts at edges)
            display = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            color = (0, 255, 255)
        else:
            # Undistorted + Cropped to ROI
            undistorted = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            x, y, w, h = roi
            display = undistorted[y:y+h, x:x+w]
            color = (0, 255, 0)
        
        # Draw label
        label = f"{mode_names[mode]} (U to cycle)"
        cv2.putText(display, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(display, f"RMS: {rms_error:.4f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        
        cv2.imshow("Calibration Visualization", display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('u'):
            mode = (mode + 1) % 3
            print(f"Mode: {mode_names[mode]}")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
