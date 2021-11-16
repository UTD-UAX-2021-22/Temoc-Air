import cv2
from cv2 import aruco

import argparse

# parser = argparse.ArgumentParser(description='Generate Charuco Board for Camera Calibration')
# parser.add_argument('--marker_size', type = int, choices=)

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
# aruco_dict.bytesList = aruco_dict.bytesList[30:,:,:]
x_dim = 8
y_dim = 10
ppi = 500
board = aruco.CharucoBoard_create(x_dim, y_dim, 1, 0.75, aruco_dict)

imboard = board.draw((x_dim*ppi, y_dim * ppi))
cv2.imwrite("chessboard1.png", imboard)