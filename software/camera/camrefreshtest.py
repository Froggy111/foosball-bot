import numpy as np
import cv2
from flask import Flask, Response
from picamera2 import Picamera2
import io
from threading import Condition

from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

import time as t
app = Flask(__name__)

mtx = np.array([[301.49190596,0.,314.35147902], [0.,301.25920544,215.49344076],[0.,0.,1.]])
dist = np.array([-0.31445176,  0.10576482,  0.00376648,  0.00188659, -0.0180011 ])

# Ball hsv vals
ball_min = [0, 0.10, 0.35] #[9, 0.60, 0.10] #hex_to_hsv(hex_color_min)
ball_max = [25, 1 , 0.95] #[13, 1, 0.75] #hex_to_hsv(hex_color_max)

# Scale HSV values for OpenCV (OpenCV uses 0-180 for H, 0-255 for S and V)
ball_min = np.array([ball_min[0] / 2, ball_min[1] * 255, ball_min[2] * 255], dtype=np.uint8)
ball_max = np.array([ball_max[0] / 2, ball_max[1] * 255, ball_max[2] * 255], dtype=np.uint8)

# Green hsv vals
green_min = np.array([80, 120, 100], dtype=np.uint8)
green_max = np.array([110, 255, 230], dtype=np.uint8)

# cX = 0
# cY = 0

# camera = Picamera2()
# camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
# camera.framerate = 32
# camera.start()
w = 640
h = 480
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)

#!/usr/bin/python3

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:8000
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


picam2 = Picamera2()
# picam2.video_configuration.controls.FrameRate = 90.0
# picam2.create_video_configuration(controls={"FrameDurationLimits": (16971, 16971)})
picam2.video_configuration.controls.FrameDurationLimits = (40000, 40000)
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)},controls={"FrameDurationLimits": (16971, 16971)}))
output = StreamingOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))
print(picam2.camera_controls)
print(picam2.camera_properties)


def generate_frames():
    while True:
        start = t.perf_counter()
        with output.condition:
            output.condition.wait()
            frame = output.frame
        frame = np.array(bytearray(frame), dtype = "uint8")
        frame = cv2.imdecode(frame, -1)
        end = t.perf_counter()
        print(end-start)


generate_frames()