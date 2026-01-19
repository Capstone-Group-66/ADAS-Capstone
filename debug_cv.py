import sys
try:
    import cv2
    print(f"OpenCV loaded: {cv2.__version__}")
    print(f"File: {cv2.__file__}")
    
    # Check attributes
    print(f"Has VideoCapture: {hasattr(cv2, 'VideoCapture')}")
    print(f"Has imshow: {hasattr(cv2, 'imshow')}")
    
    print("Attempting to open camera...")
    cap = cv2.VideoCapture(0)
    print(f"Cap created: {cap}")
    if cap is not None:
        print(f"Cap opened: {cap.isOpened()}")
        cap.release()
        
    print("Success")
except Exception as e:
    print(f"\nCRITICAL ERROR: {e}")
    import traceback
    traceback.print_exc()
