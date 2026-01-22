#!/usr/bin/env python3
"""
view_undistortion.py

Demonstration script to show the effect of camera calibration.
Loads the FrontCam_calibration.yaml and displays side-by-side
original vs undistorted video feed.

Usage:
    python3 view_undistortion.py [camera_id] [calibration_file]
"""

import cv2
import yaml
import numpy as np
import sys
import os

def load_calibration_yaml(filepath):
    """
    Load camera matrix and distortion coefficients from OpenCV-style YAML/XML.
    OpenCV's Python bindings strictly prefer XML for FileStorage, but we can parse simple YAML manually
    or try cv2.FileStorage if available.
    """
    if not os.path.exists(filepath):
        print(f"Error: Calibration file not found at {filepath}")
        return None, None, None

    # OpenCV FileStorage is the most robust way if the file was saved by OpenCV
    cv_file = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    if cv_file.isOpened():
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_coeffs = cv_file.getNode("distortion_coefficients").mat()
        width = int(cv_file.getNode("image_width").real())
        height = int(cv_file.getNode("image_height").real())
        cv_file.release()
        return camera_matrix, dist_coeffs, (width, height)
    else:
        print("Failed to open file using cv2.FileStorage")
        return None, None, None

def main():
    # Get the script's directory to resolve relative paths correctly
    script_dir = os.path.dirname(os.path.abspath(__file__))
    jetson_core_dir = os.path.dirname(script_dir)  # Go up from scripts/ to jetson-core/
    
    # Defaults
    cam_id = 0
    calib_file = os.path.join(jetson_core_dir, "config", "calibration", "FrontCam_calibration.yaml")

    if len(sys.argv) > 1:
        cam_id = int(sys.argv[1])
    if len(sys.argv) > 2:
        calib_file = sys.argv[2]
        
    print(f"Opening camera {cam_id}...")
    print(f"Loading calibration from {calib_file}...")
    
    K, D, resolution = load_calibration_yaml(calib_file)
    
    if K is None:
        print("Could not load calibration data. Exiting.")
        sys.exit(1)
        
    print("Camera Matrix:\n", K)
    print("Distortion Coeffs:\n", D)
    
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print(f"Failed to open camera {cam_id}")
        sys.exit(1)
        
    # Attempt to set resolution to match calibration
    if resolution:
        w, h = resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        print(f"Requested Resolution: {w}x{h}")
    
    # Read one frame to initialize maps
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        sys.exit(1)
        
    h, w = frame.shape[:2]
    
    # Compute optimal new camera matrix
    # alpha=0: Crop to valid pixels only (no black borders)
    # alpha=1: Keep all source pixels (has black borders)
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))
    
    # Precompute maps for remapping (faster than undistort() per frame)
    mapx, mapy = cv2.initUndistortRectifyMap(K, D, None, new_K, (w, h), 5)
    
    # Get ROI for cropping (removes black borders)
    rx, ry, rw, rh = roi
    
    print(f"\nStarting video loop. Press 'q' to quit.")
    print(f"ROI for cropping: x={rx}, y={ry}, w={rw}, h={rh}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Undistort
        undistorted = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        
        # Crop to ROI to remove any remaining black borders
        if rw > 0 and rh > 0:
            undistorted = undistorted[ry:ry+rh, rx:rx+rw]
        
        # Resize for side-by-side display if too large
        display_w = 640
        scale = display_w / w
        display_h = int(h * scale)
        
        vis_original = cv2.resize(frame, (display_w, display_h))
        vis_undistorted = cv2.resize(undistorted, (display_w, display_h))
        
        # Add labels
        cv2.putText(vis_original, "Original (Distorted)", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(vis_undistorted, "Calibrated (Undistorted)", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Stack horizontal
        combined = np.hstack((vis_original, vis_undistorted))
        
        cv2.imshow("Calibration Demo", combined)
        
        if cv2.waitKey(1) == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
