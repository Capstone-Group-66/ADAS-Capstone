import cv2
import time
import threading

# ==========================================
# CONFIGURATION: THE "FIT ON USB 2.0" SETTINGS
# ==========================================
CAMERA_INDICES = [0, 1, 2, 3]

# QVGA (320x240) is required to fit 4 cameras on one USB 2.0 Bus
WIDTH = 320
HEIGHT = 240
TARGET_FPS = 20

# MJPEG is mandatory
FOURCC = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

results = {}

def capture_thread(dev_id):
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
    
    # 1. Force MJPEG
    cap.set(cv2.CAP_PROP_FOURCC, FOURCC)
    
    # 2. Force Low Res (The only way to fit 4 cams on Bus 01)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    
    if not cap.isOpened():
        results[dev_id] = {"fps": 0, "status": "OPEN_FAIL"}
        return

    # Warmup
    for _ in range(10):
        cap.read()
        
    start_time = time.time()
    frames = []
    count = 0
    
    # Run for 3 seconds
    while (time.time() - start_time) < 3.0:
        ret, frame = cap.read()
        if ret:
            t_now = time.time()
            count += 1
            if count <= 20:
                frames.append((count, t_now))
        else:
            results[dev_id] = {"fps": 0, "status": "BANDWIDTH_CRASH"}
            break
            
    total_time = time.time() - start_time
    fps = count / total_time if total_time > 0 else 0
    
    results[dev_id] = {"fps": fps, "status": "OK", "data": frames}
    cap.release()

def main():
    print(f"--- ADAS 4-Camera QVGA Test ---")
    print(f"Resolution: {WIDTH}x{HEIGHT} (QVGA)")
    print(f"Target: {TARGET_FPS} FPS")
    print("Starting all 4 streams simultaneously...")
    print("-" * 60)

    threads = []
    for dev in CAMERA_INDICES:
        t = threading.Thread(target=capture_thread, args=(dev,))
        threads.append(t)
        t.start()
        # Tiny stagger to let the USB controller breathe
        time.sleep(0.2)

    for t in threads:
        t.join()

    print("\n" + "="*60)
    print(f"{'CAM':<5} | {'FPS':<10} | {'STATUS':<15}")
    print("="*60)
    
    for dev in CAMERA_INDICES:
        res = results.get(dev, {})
        print(f"{dev:<5} | {res.get('fps', 0):.2f}       | {res.get('status', 'UNKNOWN')}")

    print("\n" + "="*60)
    print("SAMPLE 20 FPS READING (QVGA)")
    print("="*60)
    
    for dev in CAMERA_INDICES:
        print(f"\n--- CAM {dev} ---")
        data = results.get(dev, {}).get('data', [])
        if not data:
            print("No Data")
            continue
            
        prev = data[0][1]
        print(f"{'Seq':<5} | {'Timestamp':<20} | {'Delta (ms)'}")
        for seq, t in data:
            delta = (t - prev) * 1000
            print(f"{seq:<5} | {t:<20.4f} | {delta:.1f}")
            prev = t

if __name__ == "__main__":
    main()
