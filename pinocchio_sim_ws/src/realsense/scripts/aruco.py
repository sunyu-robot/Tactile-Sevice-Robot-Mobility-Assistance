import sys
sys.path.insert(0, "/usr/local/python3/lib/python3.6/site-packages")
import cv2
import cv2.aruco as aruco
import numpy as np


class Aruco_detect:
    def __init__(self, camera_matrix, dist):
        self.camera_matrix = camera_matrix
        self.dist = dist

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.08, self.camera_matrix, self.dist)
            (rvec-tvec).any()
            for i in range(rvec.shape[0]):
                aruco.drawAxis(frame, self.camera_matrix, self.dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                aruco.drawDetectedMarkers(frame, corners)

        return corners, ids, frame