from ultralytics import YOLO
import cv2

# Model	                     Speed	    Accuracy
# yolov8n.pt (nano)	         Fastest	Low
# yolov8s.pt (small)	     Fast	    Medium
# yolov8m.pt (medium)	     Medium	    Medium-high
# yolov8l.pt (large)	     Slow	    High
# yolov8x.pt (extra-large)	 Slowest	Highest


# Load a YOLOv8 model (small or medium for testing)
model = YOLO("yolov8m.pt")  # you can use yolov8m.pt or l.pt for speed
# model = YOLO("yolov8s.pt").to("cuda").half() # move model to GPU and use half precision

cap = cv2.VideoCapture("dashcam2.mp4")  # change to your video

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)  # PyTorch + CUDA inference
    annotated_frame = results[0].plot()

    cv2.imshow("Object Detection", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
