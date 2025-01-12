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
picam2.configure(picam2.create_video_configuration(main={"format": 'XRGB8888',"size": (640, 480)},controls={"FrameDurationLimits": (10000, 20000)}))
output = StreamingOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))


def generate_frames(feed):
    cX = 0
    cY = 0
    temp = 0
    topleft = [96,33]
    topright = [232,  66]
    bottomleft = [100, 250]
    bottomright = [229, 302]
    while True:
        start = t.perf_counter()
        print(1/(start-temp))
        temp = start
        # with output.condition:
        #     output.condition.wait()
        #     frame = output.frame
        # frame = np.array(bytearray(frame), dtype = "uint8")
        # frame = cv2.imdecode(frame, -1)
        frame = picam2.capture_array()
        # print(1/(t.perf_counter()-start))
        dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        x, y, wi, he = roi
        # hsv_frame = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
        # green_mask = cv2.inRange(hsv_frame, green_min, green_max)
        # ball_mask = cv2.inRange(hsv_frame, ball_min, ball_max)
        # greencontours = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        # greenarea = 0
        # threshold = 500
        # topleftpt = [240,320]
        # toprightpt = [0,320]
        # bottomleftpt = [90,250]
        # bottomrightpt = [0,0]
        # for contour in greencontours:
        #     if cv2.contourArea(contour) > threshold:
        #         # greenarea = cv2.contourArea(contour)
        #         rect = cv2.minAreaRect(contour)
        #         boxpts = cv2.boxPoints(rect)
        #         boxpts = np.intp(boxpts)
        #         for pt in boxpts:
        #             if pt[0] - topleftpt[0] < 10 and pt[1] - topleftpt[1] < 10:
        #                 topleftpt = pt
        #             elif pt[0] - toprightpt[0] > -10 and pt[1] - toprightpt[1] < 10:
        #                 toprightpt = pt
        #             elif pt[0] - bottomleftpt[0] < 10 and pt[1] - bottomleftpt[1] > -10:
        #                 if pt[0] > 200 and pt[1] > 200:
        #                     cv2.drawContours(dst, [boxpts], 0, (255,255,0), 2)
        #                 bottomleftpt = pt
        #             elif pt[0] - bottomrightpt[0] > -10 and pt[1] - bottomrightpt[1] > -10:
        #                 bottomrightpt = pt
        # cnt = np.array([[topleftpt],
        #                  [toprightpt],
        #                  [bottomrightpt],
        #                  [bottomleftpt]])
        # cnt = np.array([[topleft],
        #                 [topright],
        #                 [bottomright],
        #                 [bottomleft]])
        # print("topleft", topleftpt)
        # print("topright", toprightpt)
        # print("bottomleft", bottomleftpt)
        # print("bottomright", bottomrightpt)
        # rect = cv2.minAreaRect(cnt)
        # convert rect to 4 points format
        # box = cv2.boxPoints(rect)
        # box = np.intp(box)

        # draw the rotated rectangle box in the image
        # cv2.drawContours(dst, [box], 0, (0, 0, 255), 2)
        # # dst = crop_minAreaRect(dst, rect)
        dst = dst[y-50:y+he+50, x:x+wi]
        dst = dst[topleft[1]-5:bottomright[1]+5,topleft[0]-5:bottomright[0]+5]
        hsv_frame2 = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
        ball_mask = cv2.inRange(hsv_frame2, ball_min, ball_max)
        ballcontours = cv2.findContours(ball_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        for c in ballcontours:
            # continue
            if cv2.contourArea(c) > 15:
                # print(cv2.contourArea(c))
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])  
        #     # draw the contour and center of the shape on the image
                # cv2.drawContours(dst, [c], -1, (0, 255, 0), 2)
        cv2.circle(dst, (cX, cY), 7, (255, 255, 255), -1)
        print(cX, cY)
        
        if feed == 1:

            ret, buffer = cv2.imencode(".jpg", dst)
            # img = np.array(buffer)
            # print(img.shape)
            # h,  w = img.shape[:2]
            frame = buffer.tobytes()
            # Yield the first stream (green_mask)
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        elif feed == 2:
            ret, buffer = cv2.imencode(".jpg", dst)
            frame = buffer.tobytes()

            
            yield (b'--frame\r\n'
                b'Content-Type: image/jpef\r\n\r\n' + frame + b'\r\n'
                )
        
        elif feed == 3:
            ret, buffer = cv2.imencode(".jpg", ball_mask)
            frame = buffer.tobytes()

            
            yield (b'--frame\r\n'
                b'Content-Type: image/jpef\r\n\r\n' + frame + b'\r\n'
                )
        # end = t.perf_counter()
        # print(1/(end-start))



@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(1), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    return Response(generate_frames(2), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed3')
def video_feed3():
    return Response(generate_frames(3), mimetype='multipart/x-mixed-replace; boundary=frame')



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

