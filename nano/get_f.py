
import os

import cv2
import numpy as np

objpoints = []
imgpoints = []
square_size = 2 * 0.01

pattern_size = (8, 6)
path = "./pic"
for root, dirs, files in os.walk(path):
    for file in files:
        if file.endswith(".png"):
            img = cv2.imread(os.path.join(root, file))
            h, w = img.shape[:2]
            print(f"{file} {w} {h}")
            objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
            objp *= square_size
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)
                img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)
                img = cv2.resize(img, (320, 240))
                cv2.imshow('Chessboard Corners', img)
                cv2.waitKey(1000)
cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("camera_matrix")
print(mtx)
print("\ndistortion factor: ")

print(dist)
