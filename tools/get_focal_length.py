import numpy as np
import cv2
import sys

def main():
    # settings
    CHECKERBOARD = (9, 6) # Inner corners
    SQUARE_SIZE = 1.0     # Relative units (results in fx, fy pixels, but T in these units)
    MIN_IMAGES = 10

    # termination criteria
    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    criteria = (3, 30, 0.001) # 3 = EPS + MAX_ITER

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * SQUARE_SIZE

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    print("Starting Camera Selector...")
    
    selected_cap = None
    for cam_idx in range(10): # Check first 10 indices
        print(f"Checking camera index {cam_idx}...")
        try:
            cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap.release()
                continue
            
            # Set resolution for preview
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # Preview loop
            print(f"Showing camera {cam_idx}. Look at the window.")
            chosen = False
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                    
                cv2.putText(frame, f"Camera Index: {cam_idx}", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame, "Press 'Y' to SELECT", (20, 80), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame, "Press 'N' to NEXT", (20, 120), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                cv2.imshow('Camera Selector', frame)
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('y') or key == ord('Y'):
                    selected_cap = cap
                    chosen = True
                    break
                elif key == ord('n') or key == ord('N'):
                    break
                elif key == ord('q'):
                    cap.release()
                    sys.exit(0)
            
            if chosen:
                break
            
            cap.release()
            
        except Exception as e:
            print(f"Error checking camera {cam_idx}: {e}")

    cv2.destroyWindow('Camera Selector')
    
    if selected_cap is None:
        print("No camera selected!")
        return
        
    cap = selected_cap
    print(f"Camera selected. Starting calibration...")

    print("Controls:")
    print("  's' - Save current frame (if corners found)")
    print(f"  'c' - Calibrate and Exit (needs {MIN_IMAGES} frames)")
    print("  'q' - Quit without calibrating")

    img_shape = None
    saved_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]

        # Find the chess board corners
        ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display_frame = frame.copy()

        if ret_corners:
            # Draw and display the corners
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_corners)
            cv2.putText(display_frame, "Corners Found! Press 's' to save.", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "Looking for chessboard...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display_frame, f"Saved: {saved_count}", (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if saved_count >= MIN_IMAGES:
            cv2.putText(display_frame, "Ready to Calibrate! Press 'c'", (20, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('Calibration Wizard', display_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            if ret_corners:
                # Refine corners
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners2)
                saved_count += 1
                print(f"Captured Image #{saved_count}")
            else:
                print("No corners found, cannot save.")
        elif key == ord('c'):
            if saved_count >= MIN_IMAGES:
                print("Calibrating... please wait...")
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
                
                print("\n" + "="*40)
                print("CALIBRATION SUCCESSFUL")
                print("="*40)
                print(f"Reprojection Error: {ret:.4f}")
                print("\nCamera Matrix (K):")
                print(mtx)
                print("\nDistortion Coefficients (D):")
                print(dist)
                
                fx = mtx[0,0]
                fy = mtx[1,1]
                cx = mtx[0,2]
                cy = mtx[1,2]

                print("\n" + "-"*40)
                print(f"Focal Length X (fx): {fx:.2f} px")
                print(f"Focal Length Y (fy): {fy:.2f} px")
                print(f"Principal Point (cx, cy): ({cx:.2f}, {cy:.2f})")
                print("-"*40)

                # Save to file
                out_file = "camera_intrinsics.yaml"
                fs = cv2.FileStorage(out_file, cv2.FILE_STORAGE_WRITE)
                fs.write("camera_matrix", mtx)
                fs.write("dist_coeffs", dist)
                fs.write("image_width", img_shape[0])
                fs.write("image_height", img_shape[1])
                fs.release()
                print(f"\nSaved to {out_file}")
                
                break
            else:
                print(f"Not enough images. Need {MIN_IMAGES}, have {saved_count}.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
