import cv2
import numpy as np

# Use the AprilTag 36h11 dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

img = cv2.imread("example_apriltag_image.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

corners, ids, rejected = detector.detectMarkers(gray)

if ids is not None:
    print(f"Detected {len(ids)} AprilTags.")
    cv2.aruco.drawDetectedMarkers(img, corners, ids)
    cv2.imshow("Detected Tags", img)
    cv2.waitKey(0)

