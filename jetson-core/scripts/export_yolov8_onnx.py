#!/usr/bin/env python3
"""
Export YOLOv8n model to ONNX format compatible with OpenCV 4.1.1

The Jetson's OpenCV 4.1.1 has a bug with int64 tensors in ONNX models.
This script exports with opset=12 and simplifies the model for compatibility.

Usage:
    pip install ultralytics
    python3 export_yolov8_onnx.py

This will create yolov8n_cv41.onnx in the current directory.
"""

from ultralytics import YOLO
import shutil

def main():
    print("[Export] Loading YOLOv8n pretrained model...")
    model = YOLO('yolov8n.pt')
    
    print("[Export] Exporting to ONNX with opset=12...")
    model.export(
        format='onnx',
        opset=12,           # Lower opset for OpenCV 4.1.1 compatibility
        simplify=True,      # Use ultralytics' built-in simplify
        dynamic=False,      # Static input shape
        imgsz=640,          # Input size
    )
    
    # The export creates yolov8n.onnx, rename for clarity
    shutil.move('yolov8n.onnx', 'yolov8n_cv41.onnx')
    
    print("[Export] Saved to yolov8n_cv41.onnx")
    print("[Export] Copy this file to jetson-core/models/yolov8n.onnx on the Jetson")

if __name__ == "__main__":
    main()
