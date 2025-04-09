import numpy as np
import cv2 as cv

# mtx = np.loadtxt("cameramatrix.txt")
# dist = np.loadtxt("distortion.txt")
# print(mtx)
# print(dist)

mtx = np.array([[301.49190596,0.,314.35147902], [0.,301.25920544,215.49344076],[0.,0.,1.]])
dist = np.array([-0.31445176,  0.10576482,  0.00376648,  0.00188659, -0.0180011 ])

img = cv.imread('test2.jpg')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

np.savetxt("cameramatrix.txt",mtx)
np.savetxt("distortion.txt", dist)
 
# crop the image
x, y, w, h = roi
offset = 0
dst = dst[y-offset:y+h+offset, x:x+w]
cv.imwrite('testing.png', dst)
