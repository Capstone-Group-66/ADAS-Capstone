import cv2
import time

def run_mjpeg_test(dev_id=0):
    print(f"--- Testing InnoMaker U20CAM-1080P on /dev/video{dev_id} ---")
    
    # 1. Open with V4L2 backend
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
    
    # 2. THE CRITICAL FIX: Force MJPEG (FourCC)
    # This prevents the camera from defaulting to the slow YUYV mode
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    # 3. Set Resolution (Try 1280x720 first to save USB bandwidth for 3 cams)
    target_w, target_h = 1280, 720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_h)
    
    # 4. Verify Settings
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"Codec Active: {codec} (Expect MJPG)")
    print(f"Resolution:   {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
    print(f"Reported FPS: {fps}")

    if codec != "MJPG":
        print("WARNING: Camera rejected MJPEG. USB Bandwidth might be full or driver issue.")

    # 5. Measure REAL FPS
    print("\nMeasuring actual FPS over 60 frames...")
    start = time.monotonic()
    count = 0
    for _ in range(60):
        ret, frame = cap.read()
        if not ret:
            break
        count += 1
    end = time.monotonic()
    
    actual_fps = count / (end - start)
    print(f"ACTUAL FPS: {actual_fps:.2f}")

    if actual_fps < 15:
        print("FAILURE: Frame rate too low for ADAS.")
    else:
        print("SUCCESS: Camera is viable for Phase 2.")
        cv2.imwrite("sample_mjpeg.jpg", frame)

    cap.release()

if __name__ == "__main__":
    run_mjpeg_test(0)
