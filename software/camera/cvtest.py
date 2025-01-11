import cv2
import numpy as np
import math as m

# Ball position vars
ballx, bally = 0,0

width, height = 300, 500

# Ball hsv vals
ball_min = [38, 0.70, 0.10] #[9, 0.60, 0.10] #hex_to_hsv(hex_color_min)
ball_max = [50, 1, 0.75] #[13, 1, 0.75] #hex_to_hsv(hex_color_max)

# Scale HSV values for OpenCV (OpenCV uses 0-180 for H, 0-255 for S and V)
ball_min = np.array([ball_min[0] / 2, ball_min[1] * 255, ball_min[2] * 255], dtype=np.uint8)
ball_max = np.array([ball_max[0] / 2, ball_max[1] * 255, ball_max[2] * 255], dtype=np.uint8)

# Green hsv vals
green_min = np.array([60, 110, 30], dtype=np.uint8)
green_max = np.array([85, 255, 155], dtype=np.uint8)

# Blank field frame
blank_image = np.zeros((width, height,3), np.uint8)

# Load the video
video = cv2.VideoCapture("video.mp4")
if not video.isOpened():
    print("Error: Could not load video.")
    exit()

# Saved video properties
frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(video.get(cv2.CAP_PROP_FPS))

# Define the codec and create VideoWriter
output_video = cv2.VideoWriter(
    "output_detected_ball.mp4",
    cv2.VideoWriter_fourcc(*"mp4v"),
    fps,
    (frame_width, frame_height)
)

while True:
    ret, frame = video.read()
    if not ret:
        break

    # Refresh frame
    blank_image = np.zeros((width, height,3), np.uint8)

    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the green color range
    green_mask = cv2.inRange(hsv_frame, green_min, green_max)

    # Create a mask for the ball's color range
    ball_mask = cv2.inRange(hsv_frame, ball_min, ball_max)

    # Find contours of the masked regions
    greencontours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ballcontours, _ = cv2.findContours(ball_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest green contour and draw it 
    greenarea = 0
    boxpts = []
    for contour in greencontours:
        if cv2.contourArea(contour) > greenarea:
            greenarea = cv2.contourArea(contour)
            # print(cv2.contourArea(contour))
            rect = cv2.minAreaRect(contour)
            boxpts = cv2.boxPoints(rect)
            boxpts = np.intp(boxpts)
    cv2.drawContours(frame,[boxpts],0,(0,0,255),2)
    print(boxpts)
    # Find the largest ball contour and draw it
    maxball = 0
    for contour in ballcontours:
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        if cv2.pointPolygonTest(boxpts,(x,y),True) > 0 and radius > maxball:
            maxball = radius
            ballx, bally = x, y
    cv2.circle(frame, (int(ballx), int(bally)), int(maxball), (0, 255, 0), 2)
    cv2.circle(frame, (int(ballx), int(bally)), 3, (0, 255, 255), -1)
    print(f"max ball radius: {maxball} ({ballx}, {bally})")

    [pt4x, pt4y] = boxpts[3]
    [pt3x, pt3y] = boxpts[2]
    [pt1x, pt1y] = boxpts[0]
    last = height
    height = int(m.sqrt((pt3x-pt4x)**2 + (pt3y-pt4y)**2))
    width = int(m.sqrt((pt1x-pt4x)**2 + (pt1y-pt4y)**2))
    print(f"width: {width} height: {height}")
    if width > height:
        last = height
        height = width
        width = last

    field = cv2.RotatedRect(center = (1,1), size =  (1,1), angle = 0.1)

    field = cv2.minAreaRect(boxpts) #cv2.RotatedRect(point1 = boxpts[0], point2 = boxpts[1], point3 = boxpts[2])
    print(type(field))
    angle_rad = m.radians(90-field[2])  #Negative to reverse rotation
    print(angle_rad)
    # Translate the point relative to the rectangle center
    # translated_point = (ballx - field[0][0], field[0][1] - bally)
    print("point4: " + str(pt1x), str(pt1y))
    print("ball: "+ str(ballx), str(bally))
    translated_point = (ballx - pt1x, bally - pt1y)
    print(translated_point)

    # Rotate the point
    x_prime = (translated_point[0] * np.cos(angle_rad) - 
               translated_point[1] * np.sin(angle_rad)) 
    y_prime = (translated_point[0] * np.sin(angle_rad) + 
               translated_point[1] * np.cos(angle_rad))
    print(x_prime, y_prime)
    cv2.circle(blank_image, (int(x_prime),int(y_prime)), 10, (0,0,255), -1)


    # Save and show the results
    output_video.write(frame)
    cv2.imshow("Detected Ball", frame)
    cv2.imshow("Field Mask", green_mask)
    cv2.imshow("Ball Track", blank_image)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Click to go frame bvy frame
    # cv2.waitKey()

# Release resources
video.release()
cv2.destroyAllWindows()
