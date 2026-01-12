# detectors/yolo_detector.py
from ultralytics import YOLO
import cv2

class YoloDetector:
    def __init__(self, model_path="yolov8m.pt", conf_threshold=0.4, device="cuda"):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.device = device

    def detect_frame(self, frame):
        results = self.model(frame)[0]  # YOLOv8 inference
        annotated_frame = results.plot()
        # Extract object info
        objects = []
        for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            objects.append({
                "bbox": box.cpu().numpy().tolist(),
                "class_id": int(cls.cpu().numpy()),
                "conf": float(conf.cpu().numpy())
            })
        return annotated_frame, objects
