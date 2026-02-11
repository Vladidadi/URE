import cv2
import numpy as np

# Define the dictionary and board
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard(
    size=(5, 7),           # number of chessboard squares (columns, rows)
    squareLength=0.04,     # square side length in meters
    markerLength=0.02,     # marker side length in meters
    dictionary=aruco_dict
)

# Generate an image of the board for printing
img = board.generateImage((800, 1000))  # ✅ Correct method name!

cv2.imwrite("charuco_board.png", img)
cv2.imshow("ChArUco Board", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
