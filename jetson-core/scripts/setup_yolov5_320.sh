#!/bin/bash
# Script to download, export, and build YOLOv5n at 320x320 resolution
# Run this on the Jetson Nano

set -e

MODEL_DIR="$(pwd)/models"
mkdir -p "$MODEL_DIR"

echo "=========================================================="
echo "  Setting up 320x320 YOLOv5n for High Performance"
echo "=========================================================="

# 1. Clone YOLOv5 repo specific tag (v7.0) to ensure compatibility
if [ ! -d "yolov5_export" ]; then
    echo "[1/3] Cloning YOLOv5 repo..."
    git clone --depth 1 --branch v7.0 https://github.com/ultralytics/yolov5 yolov5_export
else
    echo "[1/3] YOLOv5 repo already exists, skipping clone."
fi

# 2. Export ONNX at 320x320
echo "[2/3] Exporting YOLOv5n to ONNX (320x320)..."
cd yolov5_export
# Install requirements if needed (assuming user has torch/torchvision already)
# pip install -r requirements.txt  <-- might break system packages, skipping
# Just try running export
python3 export.py --weights yolov5n.pt --include onnx --img 320 --opset 12

# Move result
mv yolov5n.onnx "${MODEL_DIR}/yolov5n_320.onnx"
cd ..
echo "ONNX exported to ${MODEL_DIR}/yolov5n_320.onnx"

# 3. Convert to TensorRT
if command -v /usr/src/tensorrt/bin/trtexec &> /dev/null; then
    echo "[3/3] Building TensorRT Engine (320x320)..."
    echo "This will take a few minutes..."
    
    /usr/src/tensorrt/bin/trtexec \
        --onnx="${MODEL_DIR}/yolov5n_320.onnx" \
        --saveEngine="${MODEL_DIR}/yolov5n_320.engine" \
        --fp16 \
        --workspace=1024
        
    echo "=========================================================="
    echo "  SUCCESS! Engine built at: ${MODEL_DIR}/yolov5n_320.engine"
    echo "=========================================================="
else
    echo "WARNING: trtexec not found. ONNX is ready, but you need to run TensorRT conversion manually."
fi
