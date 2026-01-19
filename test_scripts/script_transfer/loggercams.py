import cv2
import time
import threading
import os

# ==========================================
#  GLOBAL CONFIGURATION
# ==========================================

# 1. DEFINITIONS (Width, Height)
RES_LOW = (700, 400)  # 1080p for Primary
RES_HIGH  = (1280, 720)    # VGA for Secondaries (Closest standard to 700x400)

# 2. MAPPING
# Which camera gets the High Res? (e.g., 0 = Front)
PRIMARY_CAM_ID = 0 
# List of all active cameras (e.g., 3 cameras total now)
CAMERA_INDICES = [0, 1, 2]

# 3. SYSTEM SETTINGS
TARGET_FPS = 20
TEST_DURATION_SEC = 3.0

# 4. OUTPUT
DESKTOP_PATH = os.path.expanduser("~/Desktop")
OUTPUT_FOLDER = os.path.join(DESKTOP_PATH, "adas_samples_mixed")

# ==========================================
#  INTERNAL LOGIC
# ==========================================
results = {}

def capture_stream(dev_id):
    """
    Configures camera based on whether it is Primary or Secondary.
    """
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
    
    # Force MJPEG (Critical)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    # DETERMINE RESOLUTION
    if dev_id == PRIMARY_CAM_ID:
        w, h = RES_HIGH
        role = "PRIMARY (1080p)"
    else:
        w, h = RES_LOW
        role = "SECONDARY (VGA)"

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    
    # Check what we actually got
    real_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    real_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    if not cap.isOpened() or real_w == 0:
        results[dev_id] = {"error": "Failed to Open", "role": role}
        return

    print(f"[Cam {dev_id}] Started as {role}. Negotiated: {int(real_w)}x{int(real_h)}")

    log_data = []
    frame_count = 0
    start_time = time.monotonic()
    
    # Capture Loop
    while (time.monotonic() - start_time) < TEST_DURATION_SEC:
        ret, frame = cap.read()
        t_ingest = time.time()
        
        if ret:
            frame_count += 1
            # Save first 20 frames
            if frame_count <= 20:
                filename = f"cam{dev_id}_{role.split()[0]}_seq{frame_count:02d}.jpg"
                filepath = os.path.join(OUTPUT_FOLDER, filename)
                cv2.imwrite(filepath, frame)

                log_data.append({
                    "seq": frame_count,
                    "t_ingest": t_ingest,
                    "size": frame.nbytes
                })
        else:
            results[dev_id] = {"error": "Stream Loss / Bandwidth Crash", "role": role}
            break
            
    total_time = time.monotonic() - start_time
    cap.release()
    
    actual_fps = frame_count / total_time if total_time > 0 else 0
    results[dev_id] = {
        "fps": actual_fps,
        "samples": log_data,
        "role": role,
        "resolution": f"{int(real_w)}x{int(real_h)}",
        "error": None
    }

def main():
    print(f"--- ADAS MIXED RESOLUTION TEST ---")
    print(f"Primary (High): {RES_HIGH} | Assigned to Cam {PRIMARY_CAM_ID}")
    print(f"Secondary (Low): {RES_LOW} | Assigned to Others")
    print(f"Target: {TARGET_FPS} FPS")
    
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
        print(f"Created output folder: {OUTPUT_FOLDER}")

    print("Initializing streams...")
    print("-" * 60)

    # Launch Threads
    threads = []
    for dev_id in CAMERA_INDICES:
        t = threading.Thread(target=capture_stream, args=(dev_id,))
        threads.append(t)
        time.sleep(0.2) # Stagger start to ease USB negotiation
        t.start()

    for t in threads:
        t.join()

    # REPORTING
    print("\n" + "="*80)
    print(f"{'CAM':<4} | {'ROLE':<15} | {'RES':<10} | {'FPS':<8} | {'STATUS':<10}")
    print("="*80)
    
    all_passed = True
    
    for dev_id in CAMERA_INDICES:
        res = results.get(dev_id, {})
        if res.get("error"):
            print(f"{dev_id:<4} | {res.get('role','N/A'):<15} | {'ERR':<10} | {0.0:<8} | {res['error']}")
            all_passed = False
            continue
            
        fps = res.get("fps", 0)
        status = "PASS" if fps >= (TARGET_FPS - 2.0) else "FAIL"
        if status == "FAIL": all_passed = False
            
        print(f"{dev_id:<4} | {res['role']:<15} | {res['resolution']:<10} | {fps:.2f}     | {status}")

    # LOG DUMP
    print("\n" + "="*80)
    print("SAMPLE 20-FRAME READING (TIMING VERIFICATION)")
    print("="*80)
    
    for dev_id in CAMERA_INDICES:
        res = results.get(dev_id, {})
        print(f"\n[STREAM {dev_id}: {res.get('role', 'UNKNOWN')}]")
        
        samples = res.get("samples", [])
        if not samples:
            print("No Data.")
            continue
            
        prev_t = samples[0]['t_ingest']
        print(f"{'Seq':<5} | {'Timestamp':<25} | {'Delta (ms)':<10}")
        print("-" * 50)
        for row in samples:
            delta = (row['t_ingest'] - prev_t) * 1000.0
            print(f"{row['seq']:<5} | {row['t_ingest']:<25.6f} | {delta:>6.1f}")
            prev_t = row['t_ingest']

    if all_passed:
        print(f"\nSUCCESS: All {len(CAMERA_INDICES)} cameras operational at mixed resolutions.")
        print(f"Images saved to: {OUTPUT_FOLDER}")
    else:
        print("\nFAILURE: Bandwidth exceeded or camera missing.")

if __name__ == "__main__":
    main()
