import cv2
import time
from ultralytics import YOLO

class SimpleYOLODetector:
    def __init__(self, model_path="yolov8s.pt", conf_thresh=0.4):
        self.model = YOLO(model_path)  # load YOLOv8
        self.conf_thresh = conf_thresh

    def detect_frame(self, frame):
        """
        Runs YOLO detection on a single frame.
        Returns annotated frame and list of detected objects.
        Each object: {"bbox": [x1, y1, x2, y2], "conf": float, "label": str}
        """
        results = self.model(frame)[0]  # PyTorch + CUDA
        annotated_frame = results.plot()

        objects = []
        for r in results.boxes:
            conf = float(r.conf[0])
            if conf < self.conf_thresh:
                continue
            x1, y1, x2, y2 = map(int, r.xyxy[0])
            label = self.model.names[int(r.cls[0])]
            objects.append({"bbox": [x1, y1, x2, y2], "conf": conf, "label": label})

        return annotated_frame, objects

def main(video_path="dashcam2.mp4"):
    cap = cv2.VideoCapture(video_path)
    detector = SimpleYOLODetector(model_path="yolov8m.pt")

    fps = 0
    frame_count = 0
    start_time = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        annotated_frame, objects = detector.detect_frame(frame)

        # Calculate FPS
        frame_count += 1
        if frame_count >= 30:
            end_time = time.time()
            fps = frame_count / (end_time - start_time)
            frame_count = 0
            start_time = time.time()
        cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("YOLOv8 Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
