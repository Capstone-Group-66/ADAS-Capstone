#!/usr/bin/env python3
"""
Convert ONNX model tensor format from raw_data to float_data for OpenCV 4.1.1 compatibility.

OpenCV 4.1.1 has a bug where it cannot parse ONNX tensors stored in the 'raw_data' field.
This script converts all tensors to use the explicit float_data/int64_data fields instead.

Usage:
    python scripts/fix_onnx_for_opencv41.py models/yolov5n.onnx models/yolov5n_fixed.onnx
"""

import onnx
from onnx import numpy_helper
import numpy as np
import sys

def convert_raw_data_to_typed(model_path, output_path):
    print(f"[Fix] Loading model: {model_path}")
    model = onnx.load(model_path)
    
    converted_count = 0
    
    # Process initializers (weights)
    for initializer in model.graph.initializer:
        if initializer.raw_data:
            # Convert raw_data to numpy array
            np_array = numpy_helper.to_array(initializer)
            
            # Clear raw_data
            initializer.ClearField('raw_data')
            
            # Set typed data based on dtype
            if np_array.dtype == np.float32:
                initializer.float_data.extend(np_array.flatten().tolist())
            elif np_array.dtype == np.float64:
                initializer.double_data.extend(np_array.flatten().tolist())
            elif np_array.dtype == np.int64:
                initializer.int64_data.extend(np_array.flatten().tolist())
            elif np_array.dtype == np.int32:
                initializer.int32_data.extend(np_array.flatten().tolist())
            else:
                # For other types, convert to float32
                converted = np_array.astype(np.float32)
                initializer.float_data.extend(converted.flatten().tolist())
                initializer.data_type = onnx.TensorProto.FLOAT
                
            converted_count += 1
            
    print(f"[Fix] Converted {converted_count} tensors from raw_data to typed fields")
    
    # Validate model
    try:
        onnx.checker.check_model(model)
        print("[Fix] Model validation passed")
    except Exception as e:
        print(f"[Fix] Warning: Model validation issue: {e}")
    
    # Save
    onnx.save(model, output_path)
    print(f"[Fix] Saved fixed model to: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python fix_onnx_for_opencv41.py <input.onnx> <output.onnx>")
        sys.exit(1)
        
    convert_raw_data_to_typed(sys.argv[1], sys.argv[2])
