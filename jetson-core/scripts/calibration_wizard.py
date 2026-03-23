#!/usr/bin/env python3
import cv2
import numpy as np
import argparse
import os
import sys
import datetime
import time
import json

def get_timestamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

def save_calibration(output_path, mtx, dist, rvecs, tvecs, width, height, rms, pattern_w, pattern_h, square_size):
    # Prepare data for OpenCV FileStorage format manually since cv2.FileStorage is tricky in Python to match C++ YAML exactly
    # But C++ opencv reads standard YAML.
    
    # We will write a simple YAML file that OpenCV C++ can parse.
    # Note: OpenCV's YAML format usually includes specific type tags like !!opencv-matrix
    
    fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
    
    fs.write("image_width", width)
    fs.write("image_height", height)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("rms_error", rms)
    fs.write("calibrated_at", get_timestamp())
    fs.write("pattern_width", pattern_w)
    fs.write("pattern_height", pattern_h)
    fs.write("square_size_m", square_size)
    
    fs.release()
    print(f"[SUCCESS] Saved calibration to: {output_path}")

def run_calibration(device_id, mount_name, output_dir, pattern_size, square_size, num_images_target):
    
    # Convert device_id to int if possible
    try:
        cap_id = int(device_id)
    except ValueError:
        cap_id = device_id # Path string for linux
        
    cap = cv2.VideoCapture(cap_id)
    
    if not cap.isOpened():
        print(f"[ERROR] Could not open video device {device_id}")
        return
        
    # Attempt 720p
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"Camera opened: {actual_w}x{actual_h}")
    print("---------------------------------------------------------")
    print(f"Calibration Target: {mount_name}")
    print(f"Pattern: {pattern_size[0]}x{pattern_size[1]} inner corners")
    print(f"Square Size: {square_size} meters")
    print("---------------------------------------------------------")
    print("CONTROLS:")
    print("  SPACE  - Capture image (only when pattern detected)")
    print("  Q      - Quit / Finish Capture")
    print("  ESC    - Cancel")
    print("---------------------------------------------------------")

    objp = np.zeros((pattern_size[1]*pattern_size[0], 3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp = objp * square_size
    
    object_points = [] # 3d point in real world space
    image_points = [] # 2d points in image plane.
    
    captured_count = 0
    last_capture_time = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find chess board corners
        ret_corners, corners = cv2.findChessboardCorners(gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        display = frame.copy()
        
        if ret_corners:
            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            
            # Draw
            cv2.drawChessboardCorners(display, pattern_size, corners2, ret_corners)
            
            # Green border
            cv2.rectangle(display, (0,0), (actual_w-1, actual_h-1), (0,255,0), 4)
            
            cv2.putText(display, "PATTERN DETECTED - PRESS SPACE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Searching for pattern...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.putText(display, f"Captured: {captured_count}/{num_images_target}", (50, actual_h - 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        cv2.imshow('Camera Calibration', display)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27: # ESC
            print("Cancelled.")
            cap.release()
            cv2.destroyAllWindows()
            return
            
        elif key == ord('q'):
            break
            
        elif key == 32: # SPACE
             if ret_corners:
                current_time = time.time()
                if current_time - last_capture_time < 1.0:
                    print("Too fast! Waiting...")
                    continue
                    
                print(f"Capturing image {captured_count+1}...")
                image_points.append(corners2)
                object_points.append(objp)
                captured_count += 1
                last_capture_time = current_time
                
                # Flash effect
                white = np.ones_like(display) * 255
                cv2.imshow('Camera Calibration', white)
                cv2.waitKey(50)
                
                if captured_count >= num_images_target:
                    print("Target images reached! Press Q to calculate or continue capturing.")

    cap.release()
    cv2.destroyAllWindows()
    
    if captured_count < 5:
        print("[ERROR] Need at least 5 images to calibrate. Exiting.")
        return

    print("Calibrating... this may take a moment...")
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, (actual_w, actual_h), None, None)
    
    if ret:
        print(f"Calibration successful! RMS Error: {ret:.4f}")
        
        # Calculate Reprojection Error
        mean_error = 0
        for i in range(len(object_points)):
            imgpoints2, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "Total Reprojection Error: {}".format(mean_error/len(object_points)) )
        
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        output_file = os.path.join(output_dir, f"{mount_name}_calibration.yaml")
        save_calibration(output_file, mtx, dist, rvecs, tvecs, actual_w, actual_h, ret, pattern_size[0], pattern_size[1], square_size)
    else:
        print("Calibration failed.")

def list_cameras():
    print("Searching for cameras...")
    available = []
    # Check first 10 indices
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                print(f"  [{i}] Generic USB Camera ({w}x{h})")
                available.append(i)
            cap.release()
    return available

if __name__ == "__main__":
    
    print("=========================================")
    print("   PYTHON CALIBRATION WIZARD (Fallback)  ")
    print("=========================================")
    
    # Lists cams
    cams = list_cameras()
    if not cams:
        print("No cameras found!")
        sys.exit(1)
        
    print("\nSelect camera index:")
    try:
        idx_str = input(f"Enter ID {cams}: ")
        dev_id = int(idx_str)
    except:
        print("Invalid selection")
        sys.exit(1)
        
    print("\nMount Name (final build): FrontCam")
    mount_name = "FrontCam"
    
    # Defaults (matches physical chessboard: 9x6 squares = 8x5 inner corners, 30mm)
    pattern_w = 8
    pattern_h = 5
    square_size = 0.030 # 30mm
    
    print(f"\nConfiguration: {mount_name} on Device {dev_id}")
    print(f"Chessboard: {pattern_w}x{pattern_h}, Square: {square_size}m")
    
    run_calibration(dev_id, mount_name, "config/calibration", (pattern_w, pattern_h), square_size, 15)
