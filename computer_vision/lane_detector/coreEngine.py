"""
coreEngine.py — ONNX inference engine wrapper for ufldDetector.
Place this file in the same directory as videoDetection.py / imageDetection.py.
"""

import numpy as np
import onnxruntime as ort


class OnnxEngine:
    def __init__(self, model_path: str):
        # Use GPU (CUDA) with CPU fallback
        self.session = ort.InferenceSession(
            model_path, providers=["CUDAExecutionProvider", "CPUExecutionProvider"]
        )

        self.framework_type = "ONNX"
        self.providers = self.session.get_providers()

        self._input_details  = self.session.get_inputs()
        self._output_details = self.session.get_outputs()

    # shape / dtype helpers

    def get_engine_input_shape(self):
        shape = self._input_details[0].shape
        # Replace dynamic axes (None / strings) with sensible defaults
        shape = [s if isinstance(s, int) and s > 0 else 1 for s in shape]
        return shape

    @property
    def engine_dtype(self):
        onnx_type = self._input_details[0].type          # ex: "tensor(float)"
        if "float16" in onnx_type:
            return np.float16
        return np.float32

    def get_engine_output_shape(self):
        shapes = [o.shape for o in self._output_details]
        names  = [o.name  for o in self._output_details]
        return shapes, names

    # inference 
    
    def engine_inference(self, input_tensor: np.ndarray):
        input_name = self._input_details[0].name
        outputs = self.session.run(None, {input_name: input_tensor})
        return outputs


class TensorRTEngine:
    """Stub — only needed if you have .trt model files."""
    def __init__(self, model_path: str):
        raise NotImplementedError(
            "TensorRT is not supported in this setup. Use an ONNX model instead."
        )
