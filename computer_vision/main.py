from object_detector.yolo_detector import YoloDetector
from object_detector.video_utils import VideoReader
import cv2
from object_detector.distance_measure import SingleCamDistanceMeasure

# Video file path
video_path = "object_detector/models/dashcam2.mp4"

# Initialize YOLO detector
detector = YoloDetector(model_path="object_detector/models/yolov8m.pt", conf_threshold=0.4, device="cuda")

# Initialize video reader
video = VideoReader(video_path, window_name="YOLO Detection")

# Initialize distance measure
distancer = SingleCamDistanceMeasure(object_list=["person", "car", "truck"])

while True:
    ret, frame = video.read_frame()
    if not ret:
        break

    # Run detection
    annotated_frame, objects = detector.detect_frame(frame)

    # Optionally: print detected objects
    #print(objects)



    # Update distances
    distancer.updateDistance(objects)
    # Optionally: detect collision inside lane polygon
    collision = distancer.calcCollisionPoint(lane_polygon)
    if collision:
        print(f"Collision risk detected at distance: {collision[2]:.2f} m")

    # Draw distance info
    distancer.DrawDetectedOnFrame(annotated_frame)



    # Display frame
    video.show_frame(annotated_frame)

    # Optional: display FPS
    fps = video.get_fps()
    if fps:
        print(f"FPS: {fps:.2f}")

    # Stop on 'q' key
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video.release()
