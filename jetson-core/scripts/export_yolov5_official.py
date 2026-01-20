#!/usr/bin/env python3
"""
Export YOLOv5n using Ultralytics' official export which handles OpenCV compatibility.
This uses their internal export.py which is battle-tested on Jetson.
"""

import subprocess
import sys
import os

def main():
    # Clone yolov5 repo if not exists
    if not os.path.exists('yolov5'):
        print("[Export] Cloning YOLOv5 repository...")
        subprocess.run(['git', 'clone', 'https://github.com/ultralytics/yolov5.git', '--depth', '1'], check=True)
    
    # Change to yolov5 directory
    os.chdir('yolov5')
    
    # Install requirements
    print("[Export] Installing YOLOv5 requirements...")
    subprocess.run([sys.executable, '-m', 'pip', 'install', '-r', 'requirements.txt', '-q'], check=True)
    
    # Export using official script with opset 10
    # The official export.py handles all the edge cases for older OpenCV
    print("[Export] Exporting YOLOv5n with official script (opset=10)...")
    subprocess.run([
        sys.executable, 'export.py',
        '--weights', 'yolov5n.pt',
        '--include', 'onnx',
        '--opset', '10',
        '--simplify',        # Simplify the model graph
        '--imgsz', '640',
    ], check=True)
    
    # Move the exported model
    os.chdir('..')
    import shutil
    shutil.move('yolov5/yolov5n.onnx', 'yolov5n_official.onnx')
    
    print("[Export] Saved to yolov5n_official.onnx")
    print("[Export] Now run: python scripts/fix_onnx_for_opencv41.py yolov5n_official.onnx models/yolov5n.onnx")

if __name__ == "__main__":
    main()
