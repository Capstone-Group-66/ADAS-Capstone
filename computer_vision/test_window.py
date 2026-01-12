# test_window.py
from ultralytics import YOLO
import cv2

# Load the YOLOv8 model
model = YOLO("object-detector/models/yolov8m.pt")

# Load your video
cap = cv2.VideoCapture("object-detector/models/dashcam2.mp4")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("End of video or cannot read frame")
        break

    # Run detection
    results = model(frame)
    annotated_frame = results[0].plot()

    # Show the frame in a window
    cv2.imshow("Object Detection", annotated_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
