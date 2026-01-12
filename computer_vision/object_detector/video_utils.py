# utils/video_utils.py
import cv2
import time

class VideoReader:
    def __init__(self, video_path, window_name="Video", save_path=None, fps=30):
        self.cap = cv2.VideoCapture(video_path)
        self.window_name = window_name
        self.save_path = save_path
        self.fps = fps
        self.frame_count = 0
        self.start_time = time.time()
        self.writer = None

        if save_path:
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(save_path, fourcc, fps, (width, height))

    def read_frame(self):
        ret, frame = self.cap.read()
        return ret, frame

    def show_frame(self, frame):
        cv2.imshow(self.window_name, frame)
        if self.writer:
            self.writer.write(frame)
        self.frame_count += 1

    def get_fps(self):
        if self.frame_count >= 30:
            end_time = time.time()
            fps = self.frame_count / (end_time - self.start_time)
            self.frame_count = 0
            self.start_time = time.time()
            return fps
        return None

    def release(self):
        self.cap.release()
        if self.writer:
            self.writer.release()
        cv2.destroyAllWindows()
