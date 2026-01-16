import cv2
import time
import numpy as np
from ultralytics import YOLO

# Configuration
VIDEO_SOURCE = "dashcam6.mp4"  # 0 = webcam
CONF_THRESHOLD = 0.4
MODEL_PATH = "yolov8n.pt"
SMOOTHING_ALPHA = 0.7  # Higher = smoother Vy

# Load YOLOv8 Nano
model = YOLO(MODEL_PATH)

# Video Capture
cap = cv2.VideoCapture(VIDEO_SOURCE)
fps = cap.get(cv2.CAP_PROP_FPS)
if fps <= 0:
    fps = 30.0  # fallback

# Object State Storage
# track_id -> previous centroid y
prev_centroid_y = {}
# track_id -> smoothed Vy
prev_vy = {}

# Main Loop
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    current_time = time.time()

    # YOLOv8 + ByteTrack
    results = model.track(
        frame,
        persist=True,
        conf=CONF_THRESHOLD,
        tracker="bytetrack.yaml",
        verbose=False
    )

    if results[0].boxes.id is not None:
        boxes = results[0].boxes

        for box, track_id in zip(boxes.xyxy, boxes.id):
            track_id = int(track_id.item())

            x1, y1, x2, y2 = map(int, box.tolist())
            w = x2 - x1
            h = y2 - y1

            # Centroid
            cx = int(x1 + w / 2)
            cy = int(y1 + h / 2)

            # Vertical velocity Vy
            Vy = 0.0

            if track_id in prev_centroid_y:
                Vy_raw = (cy - prev_centroid_y[track_id]) * fps

                # Exponential smoothing
                if track_id in prev_vy:
                    Vy = (SMOOTHING_ALPHA * prev_vy[track_id] + (1 - SMOOTHING_ALPHA) * Vy_raw)
                else:
                    Vy = Vy_raw
            else:
                Vy = 0.0

            # Store current values
            prev_centroid_y[track_id] = cy
            prev_vy[track_id] = Vy

            # Draw Bounding Box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            label = f"ID:{track_id}  Y:{cy}  Vy:{Vy:.1f}px/s"
            cv2.putText(
                frame,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            # Draw centroid
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

    # Display
    cv2.imshow("YOLOv8 Object Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
