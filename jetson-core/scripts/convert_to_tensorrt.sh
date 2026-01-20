#!/bin/bash
# Convert YOLOv5 ONNX model to TensorRT engine
# Run this script ON THE JETSON after pulling the code

set -e

MODEL_DIR="models"
ONNX_MODEL="${MODEL_DIR}/yolov5n.onnx"
ENGINE_MODEL="${MODEL_DIR}/yolov5n.engine"

echo "============================================="
echo "  ONNX → TensorRT Conversion"
echo "============================================="

# Check if ONNX model exists
if [ ! -f "$ONNX_MODEL" ]; then
    echo "ERROR: ONNX model not found at $ONNX_MODEL"
    echo "Make sure you have pulled the latest code."
    exit 1
fi

# Check if trtexec exists
if ! command -v /usr/src/tensorrt/bin/trtexec &> /dev/null; then
    echo "ERROR: trtexec not found. Is TensorRT installed?"
    exit 1
fi

echo "Converting $ONNX_MODEL to TensorRT engine..."
echo "This may take 5-10 minutes on Jetson Nano..."

/usr/src/tensorrt/bin/trtexec \
    --onnx="$ONNX_MODEL" \
    --saveEngine="$ENGINE_MODEL" \
    --fp16 \
    --workspace=1024

echo ""
echo "============================================="
echo "  Conversion Complete!"
echo "============================================="
echo "Engine saved to: $ENGINE_MODEL"
echo ""
echo "You can now run: make -j4 && ./build/jetson_core"
