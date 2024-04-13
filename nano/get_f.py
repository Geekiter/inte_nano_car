# 遍历pic目录下所有jpg图片，获取其相机内参

import os

import cv2
import numpy as np

objpoints = []  # 3D 点在世界坐标系中的坐标
imgpoints = []  # 2D 点在图像平面的坐标
# 棋盘格尺寸（单位：米）
square_size = 2 * 0.01  # 棋盘格方块的实际边长

# 棋盘格角点数目
pattern_size = (8, 6)  # 棋盘格内角点（列数，行数）
path = "./pic"
for root, dirs, files in os.walk(path):
    for file in files:
        if file.endswith(".png"):
            img = cv2.imread(os.path.join(root, file))
            h, w = img.shape[:2]
            print(f"{file} {w} {h}")
            # 棋盘格角点坐标
            objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
            objp *= square_size
            # 转换为灰度图像
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            # 如果找到角点，添加到列表中
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)
                # 绘制并显示棋盘格角点
                img = cv2.drawChessboardCorners(img, pattern_size, corners, ret)
                img = cv2.resize(img, (320, 240))
                cv2.imshow('Chessboard Corners', img)
                cv2.waitKey(1000)
# 关闭窗
cv2.destroyAllWindows()
# 标定相机
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# 打印相机参数
print("相机内参矩阵:")
print(mtx)
print("\n畸变系数:")

print(dist)
