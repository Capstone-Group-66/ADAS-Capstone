"""
Lane Detection and Lane Departure Warning Module
Extracted from Advanced Driver-Assistance System
Author: Kostantinos Katsanos
"""

import cv2
import numpy as np
from threading import Thread
import os

# Configuration Variables
starting_horizon_Ratio = 3/5  # Starting horizon line percentage of image
glassGradStreght = 65  # (0-255) Strength of gradient for windshield brightness
hood = 10  # (in pixels) Space at bottom of image taken by hood
fov = -250  # (in pixels) Starting point of lane recognition
memoryframes = 10  # (in frames) Frames that a recognized line will be kept
carsize = 500
wheelOff = 50

newLineW = 0.055 #used to be 0.2
smothBrightfact = 0.2
smothLaneCenter = 0.6
laneLockThresh = 10
lane_depth = 1.15
brightsens = 5
camOffset = 0


class LaneDeparture:
    def __init__(self, cap):
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.heightcrp = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if self.heightcrp > self.width:
            self.height = int(self.width * 9 / 16)
        else:
            self.height = self.heightcrp - hood

        self.horizon = self.height * starting_horizon_Ratio
        self.stathorizon = self.horizon
        self.lanecenter = self.width / 2
        self.glassGradient = self.createGlassGradient()
        
        self.timerl = 0
        self.timerr = 0
        self.LLFrames = 0
        self.RLFrames = 0
        self.reachedleft = False
        self.reachedright = False
        self.averaged_lines = [[[0, 0, 0, 0]], [[0, 0, 0, 0]]]
        self.laneframe = []
        
        self.color_MIN = np.array([0, 0, 150], np.uint8)
        self.color_MAX = np.array([179, 255, 255], np.uint8)


        # --- Camera calibration ---
        self.use_calibration = True
        calib_path = "jetson-core/config/calibration/FrontCam_calibration.yaml"

        try:
            fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)

            if not fs.isOpened():
                raise IOError("Failed to open calibration file")

            self.cam_mtx = fs.getNode("camera_matrix").mat()
            self.dist_coeffs = fs.getNode("distortion_coefficients").mat()

            self.calib_width = int(fs.getNode("image_width").real())
            self.calib_height = int(fs.getNode("image_height").real())

            fs.release()

            self.new_cam_mtx, _ = cv2.getOptimalNewCameraMatrix(
                self.cam_mtx,
                self.dist_coeffs,
                (self.calib_width, self.calib_height),
                alpha=0
            )

            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.cam_mtx,
                self.dist_coeffs,
                None,
                self.new_cam_mtx,
                (self.calib_width, self.calib_height),
                cv2.CV_16SC2
            )

            print("[LaneDetector] Camera calibration loaded successfully")
            print("Camera matrix:\n", self.cam_mtx)
            print("Dist coeffs:\n", self.dist_coeffs)
        except Exception as e:
            print("[LaneDetector] Calibration load failed:", e)
            self.use_calibration = False

    def get_gradient_2d(self, start, stop, width, height, is_horizontal):
        if is_horizontal:
            return np.tile(np.linspace(start, stop, width), (height, 1))
        else:
            return np.tile(np.linspace(start, stop, height), (width, 1)).T

    def get_gradient_3d(self, width, height, start_list, stop_list, is_horizontal_list):
        result = np.zeros((height, width, len(start_list)), dtype=np.uint8)
        for i, (start, stop, is_horizontal) in enumerate(zip(start_list, stop_list, is_horizontal_list)):
            result[:, :, i] = self.get_gradient_2d(start, stop, width, height, is_horizontal)
        return result

    def createGlassGradient(self):
        array = self.get_gradient_3d(
            self.width, self.heightcrp,
            (0, 0, 0),
            (glassGradStreght, glassGradStreght, glassGradStreght),
            (False, False, False)
        )
        return array

    def make_points(self, image, line):
        slope, intercept = line
        y1 = int(self.height)
        y2 = int(self.horizon + self.horizon * 15 / 100)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, image, lines):
        left_fit = []
        right_fit = []
        Rlane = image.shape[1] / 2 + 50
        Llane = image.shape[1] / 2 - 50

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    slope = (y1 - y2) / (x1 - x2)
                    if 7 > slope > 0.4: #here
                        if x1 > Rlane:
                            yintercept = y2 - (slope * x2)
                            right_fit.append((slope, yintercept))
                    elif -7 < slope < -0.4: #here
                        if x1 < Llane:
                            yintercept = y2 - (slope * x2)
                            left_fit.append((slope, yintercept))

        if left_fit:
            left_fit_average = np.average(left_fit, axis=0)
            left_line = self.make_points(image, left_fit_average)
            self.timerl = 0
        else:
            if self.timerl <= memoryframes:
                left_line = self.averaged_lines[0]
                self.timerl += 1
            else:
                left_line = [[0, 0, 0, 0]]
                self.timerl = 0

        if right_fit:
            right_fit_average = np.average(right_fit, axis=0)
            right_line = self.make_points(image, right_fit_average)
            self.timerr = 0
        else:
            if self.timerr <= memoryframes:
                right_line = self.averaged_lines[1]
                self.timerr += 1
            else:
                right_line = [[0, 0, 0, 0]]
                self.timerr = 0

        oldLineW = 1 - newLineW
        
        if any(left_line[0]):
            if any(self.averaged_lines[0][0]):
                self.averaged_lines[0][0][0] = int(self.averaged_lines[0][0][0] * oldLineW + left_line[0][0] * newLineW)
                self.averaged_lines[0][0][1] = int(self.averaged_lines[0][0][1] * oldLineW + left_line[0][1] * newLineW)
                self.averaged_lines[0][0][2] = int(self.averaged_lines[0][0][2] * oldLineW + left_line[0][2] * newLineW)
                self.averaged_lines[0][0][3] = int(self.averaged_lines[0][0][3] * oldLineW + left_line[0][3] * newLineW)
            else:
                self.averaged_lines[0] = left_line
        else:
            self.averaged_lines[0] = left_line

        if any(right_line[0]):
            if any(self.averaged_lines[1][0]):
                self.averaged_lines[1][0][0] = int(self.averaged_lines[1][0][0] * oldLineW + right_line[0][0] * newLineW)
                self.averaged_lines[1][0][1] = int(self.averaged_lines[1][0][1] * oldLineW + right_line[0][1] * newLineW)
                self.averaged_lines[1][0][2] = int(self.averaged_lines[1][0][2] * oldLineW + right_line[0][2] * newLineW)
                self.averaged_lines[1][0][3] = int(self.averaged_lines[1][0][3] * oldLineW + right_line[0][3] * newLineW)
            else:
                self.averaged_lines[1] = right_line
        else:
            self.averaged_lines[1] = right_line

        return self.averaged_lines

    def canny(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blur1 = cv2.GaussianBlur(gray, (5, 5), 0, 0)
        canny = cv2.Canny(blur1, 80, 140)
        return canny

    def get_intersect(self, averaged_lines):
        if averaged_lines == None:
            return 0, 0

        a1 = [averaged_lines[0][0][0], averaged_lines[0][0][1]]
        a2 = [averaged_lines[0][0][2], averaged_lines[0][0][3]]
        b1 = [averaged_lines[1][0][0], averaged_lines[1][0][1]]
        b2 = [averaged_lines[1][0][2], averaged_lines[1][0][3]]

        s = np.vstack([a1, a2, b1, b2])
        h = np.hstack((s, np.ones((4, 1))))
        l1 = np.cross(h[0], h[1])
        l2 = np.cross(h[2], h[3])
        x, y, z = np.cross(l1, l2)

        if z == 0:  # lines are parallel
            sl = np.vstack([a1, a2, [0, self.horizon], [self.width, self.horizon]])
            hl = np.hstack((sl, np.ones((4, 1))))
            ll1 = np.cross(hl[0], hl[1])
            hl2 = np.cross(hl[2], hl[3])
            xl, yl, zl = np.cross(ll1, hl2)

            sr = np.vstack([b1, b2, [0, self.horizon], [self.width, self.horizon]])
            hr = np.hstack((sr, np.ones((4, 1))))
            rl1 = np.cross(hr[0], hr[1])
            hl2 = np.cross(hr[2], hr[3])
            xr, yr, zr = np.cross(rl1, hl2)

            if zl != 0:
                return int((self.horizon + self.stathorizon) / 2), int(xl / zl)
            elif zr != 0:
                return int((self.horizon + self.stathorizon) / 2), int(xr / zr)
            else:
                return int((self.horizon + self.stathorizon) / 2), int((self.lanecenter + self.width / 2) / 2)

        return int(y / z), int(x / z)

    def checklaneDeparture(self, img, averaged_lines, carSpeed=0, tsign_left=False, tsign_right=False):
        mask = np.zeros_like(img)
        carPathSpace = np.array([[
            (self.width / 2 - carsize / 2 + camOffset, self.height + wheelOff),
            (self.lanecenter, self.horizon),
            (self.lanecenter, self.horizon),
            (self.width / 2 + carsize / 2 + camOffset, self.height + wheelOff)
        ]], np.int32)

        mask_limit = np.array([[
            (0, 0),
            (0, self.horizon * lane_depth),
            (self.width, self.horizon * lane_depth),
            (self.width, 0)
        ]], np.int32)

        cv2.fillPoly(mask, carPathSpace, color=(0, 0, 255))
        cv2.fillPoly(mask, mask_limit, 0)

        a1 = (averaged_lines[0][0][0], averaged_lines[0][0][1])
        resultLeftBot = cv2.pointPolygonTest(carPathSpace, a1, False)
        b1 = (averaged_lines[1][0][0], averaged_lines[1][0][1])
        resultRightBot = cv2.pointPolygonTest(carPathSpace, b1, False)

        if averaged_lines[0] != [[0, 0, 0, 0]]:
            if self.LLFrames > laneLockThresh:
                self.reachedleft = True
            else:
                self.LLFrames = self.LLFrames + 1
        else:
            if self.LLFrames > 0:
                self.LLFrames = self.LLFrames - 1
        if self.LLFrames == 0:
            self.reachedleft = False

        if averaged_lines[1] != [[0, 0, 0, 0]]:
            if self.RLFrames > laneLockThresh:
                self.reachedright = True
            else:
                self.RLFrames = self.RLFrames + 1
        else:
            if self.RLFrames > 0:
                self.RLFrames = self.RLFrames - 1
        if self.RLFrames == 0:
            self.reachedright = False

        # Lane departure warnings
        if self.reachedright and not tsign_right and carSpeed > 20:
            if resultRightBot > 0:
                cv2.putText(mask, 'LINE CROSSED (RIGHT)', (30, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.putText(mask, 'R', (60, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                       2, (0, 255, 0), 3, cv2.LINE_AA)

        if self.reachedleft and not tsign_left and carSpeed > 20:
            if resultLeftBot > 0:
                cv2.putText(mask, 'LINE CROSSED (LEFT)', (30, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.putText(mask, 'L', (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                       2, (0, 255, 0), 3, cv2.LINE_AA)

        if averaged_lines is None:
            return np.zeros_like(img)
        else:
            return mask

    def display_lines(self, img, lines):
        line_image = np.zeros_like(img)
        self.width = img.shape[1]
        
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        
        cv2.line(line_image, (1, int(self.horizon)), 
                (self.width, int(self.horizon)), (0, 255, 0), 1)
        cv2.line(line_image, (1, int(self.stathorizon)), 
                (self.width, int(self.stathorizon)), (0, 255, 20), 1)
        cv2.line(line_image, (int(self.lanecenter), 1), 
                (int(self.lanecenter), int(self.height)), (0, 255, 0), 1)
        
        return line_image

    def region_of_interest(self, canny):
        self.width = canny.shape[1]
        mask = np.zeros_like(canny)

        roi = np.array([[
            (fov + camOffset, self.height - hood),
            (self.lanecenter, self.horizon),
            (self.width / 2 - carsize / 7 + camOffset, self.height - hood),
            (self.width / 2 + carsize / 7 + camOffset, self.height - hood),
            (self.lanecenter, self.horizon),
            (self.width - fov + camOffset, self.height - hood)
        ]], np.int32)

        mask_limit = np.array([[
            (0, 0),
            (0, self.horizon * lane_depth),
            (self.width, self.horizon * lane_depth),
            (self.width, 0)
        ]], np.int32)

        cv2.fillPoly(mask, roi, 255)
        cv2.fillPoly(mask, mask_limit, 0)
        masked_image = cv2.bitwise_and(canny, mask)
        
        return masked_image, mask

    def get_road_brightness(self, hsv_img):
        ymin = int(self.horizon + (self.height - self.horizon) * 50 / 100) - 1
        ymax = int(self.height - hood) + 1
        xmin = int(self.lanecenter - carsize / 5) - 1
        xmax = int(self.lanecenter + carsize / 5) + 1
        
        masked_image = hsv_img[ymin:ymax, xmin:xmax]
        gray = cv2.cvtColor(hsv_img, cv2.COLOR_RGB2GRAY)
        brightmask = np.zeros_like(gray)

        mask = np.array([[
            (xmin, ymin),
            (xmax, ymin),
            (xmax, ymax),
            (xmin, ymax)
        ]], np.int32)

        cv2.fillPoly(brightmask, mask, 255)
        
        average_color_per_row = np.max(masked_image, axis=1, initial=1)
        average_color = np.average(average_color_per_row, axis=0)
        average_color = np.uint8(average_color)
        road_bright = average_color[2]
        
        return road_bright + brightsens, brightmask

    def laneDeparture(self, frame, carSpeed=0, tsign_left=False, tsign_right=False, debugDisplay=True):
        """
        Main lane departure detection function
        
        Args:
            frame: Input video frame
            carSpeed: Current vehicle speed (default 0)
            tsign_left: Left turn signal status (default False)
            tsign_right: Right turn signal status (default False)
            debugDisplay: Whether to create debug visualization (default True)
        
        Returns:
            combo_image: Frame with lane detection visualization if debugDisplay=True
        """
        # --- Undistort frame using camera calibration ---
        if self.use_calibration:
            if frame.shape[1] != self.calib_width or frame.shape[0] != self.calib_height:
                frame = cv2.resize(frame, (self.calib_width, self.calib_height))

            frame = cv2.remap(
                frame,
                self.map1,
                self.map2,
                interpolation=cv2.INTER_LINEAR
            )


        if self.use_calibration:
            if frame.shape[1] != self.calib_width or frame.shape[0] != self.calib_height:
                frame = cv2.resize(frame, (self.calib_width, self.calib_height))
            
            original = frame.copy()
            frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

            if debugDisplay:
                cv2.imshow("Original | Undistorted", np.hstack((original, frame)))


        if self.heightcrp > self.width:
            frame = frame[int(self.heightcrp / 2 - self.height / 2 - hood):
                         int(self.heightcrp / 2 + self.height / 2 - hood), ]
        
        frame = cv2.addWeighted(frame, 0.8, self.glassGradient, 1, 1)
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        roadbright, brightmask = self.get_road_brightness(hsv_img)
        
        self.color_MIN = np.array([0, 0, self.color_MIN[2] * (1 - smothBrightfact) + 
                                   smothBrightfact * roadbright], np.uint8)
        
        frame_threshedW = cv2.inRange(hsv_img, self.color_MIN, self.color_MAX)
        resultWfr = cv2.bitwise_and(frame, frame, mask=frame_threshedW)
        canny_image = self.canny(resultWfr)
        cropped, roi = self.region_of_interest(canny_image)
        
        lines = cv2.HoughLinesP(cropped, 2, np.pi / 180, 100, 
                               np.array([]), minLineLength=60, maxLineGap=100) #here
        
        self.average_slope_intercept(frame, lines)
        departure = self.checklaneDeparture(frame, self.averaged_lines, carSpeed, tsign_left, tsign_right)
        newhorizon, newlanecenter = self.get_intersect(self.averaged_lines)
        
        if newhorizon != 0:
            self.horizon = newhorizon
            self.stathorizon = self.stathorizon * 0.99 + self.horizon * 0.01
            self.lanecenter = self.lanecenter * (1 - smothLaneCenter) + newlanecenter * smothLaneCenter
        else:
            self.stathorizon = self.stathorizon * 0.99 + self.height * starting_horizon_Ratio * 0.01

        if debugDisplay:
            line_image = self.display_lines(frame, self.averaged_lines)
            combo_image = cv2.addWeighted(frame, 0.8, departure, 1, 1)
            combo_image = cv2.addWeighted(combo_image, 1, cv2.bitwise_and(
                combo_image, combo_image, mask=cropped), 1, 0.1)
            combo_image = cv2.addWeighted(combo_image, 1, cv2.bitwise_and(
                combo_image, combo_image, mask=roi), 0.3, 1)
            combo_image = cv2.addWeighted(combo_image, 1, cv2.bitwise_and(
                combo_image, combo_image, mask=brightmask), 0.3, 1)
            combo_image = cv2.addWeighted(combo_image, 1, line_image, 1, 1)
            
            self.laneframe = combo_image
            return combo_image
        
        return frame


# Example usage
if __name__ == "__main__":
    # Open video capture
    cap = cv2.VideoCapture("dashcam6.mp4")  # Use 0 for webcam or provide video file path
    
    # Initialize lane departure detection and locking resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    lane_detector = LaneDeparture(cap)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Process frame
        result_frame = lane_detector.laneDeparture(
            frame, 
            carSpeed=50,  # Example speed
            tsign_left=False, 
            tsign_right=False,
            debugDisplay=True
        )
        
        # Display result
        cv2.imshow('Lane Detection', result_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()