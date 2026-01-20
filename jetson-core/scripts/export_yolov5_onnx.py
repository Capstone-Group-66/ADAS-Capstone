#!/usr/bin/env python3
"""
Export YOLOv5n model to ONNX format compatible with OpenCV 4.1.1 (Jetson Nano)
Requires: pip install -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
"""

import torch
import shutil
import os

def main():
    print("[Export] Downloading YOLOv5n pretrained model...")
    # Load model from torch hub
    # 'autoshape=False' prevents wrapping in the pre-processing class
    model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True, autoshape=False)
    
    # Ensure eval mode
    model.eval()
    
    print("[Export] Exporting to ONNX with opset=10 (Legacy support)...")
    
    # Create dummy input
    dummy_input = torch.randn(1, 3, 640, 640)
    
    output_path = "yolov5n_cv41.onnx"
    
    # Export raw model
    torch.onnx.export(
        model, 
        dummy_input, 
        output_path,
        opset_version=10,       # Opset 10 for OpenCV 4.1
        do_constant_folding=True,
        input_names=['images'], # Matches YOLO standard
        output_names=['output'],
        dynamic_axes=None       # Static shape is safer
    )
    
    # Rename to standard name if needed (redundant here but keeps logic)
    print(f"[Export] Saved to {output_path}")
    print(f"[Export] Copy this file to jetson-core/models/yolov5n.onnx on the Jetson")

if __name__ == "__main__":
    main()
