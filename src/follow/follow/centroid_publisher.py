#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

#!/usr/bin/env python
 
'''
Welcome to the Object Tracking Program!
 
Using real-time streaming video from your built-in webcam, this program:
  - Creates a bounding box around a moving object
  - Calculates the coordinates of the centroid of the object
  - Tracks the centroid of the object
 
Author:
  - Addison Sears-Collins
  - https://automaticaddison.com
'''
 
#from __future__ import print_function # Python 2/3 compatibility
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
 
# Project: Object Tracking
# Author: Addison Sears-Collins 
# Website: https://automaticaddison.com
# Date created: 06/13/2020
# Python version: 3.7
 
# def main():
#     """
#     Main method of the program.
#     """
 
    # Create a VideoCapture object

 
    # while(True):
 
    #     # Capture frame-by-frame
    #     # This method returns True/False as well
    #     # as the video frame.
    #     ret, frame = cap.read()
 
    #     # Use every frame to calculate the foreground mask and update
    #     # the background
    #     fg_mask = back_sub.apply(frame)
 
    #     # Close dark gaps in foreground object using closing
    #     fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
 
    #     # Remove salt and pepper noise with a median filter
    #     fg_mask = cv2.medianBlur(fg_mask, 5) 
         
    #     # Threshold the image to make it either black or white
    #     _, fg_mask = cv2.threshold(fg_mask,127,255,cv2.THRESH_BINARY)
 
    #     # Find the index of the largest contour and draw bounding box
    #     fg_mask_bb = fg_mask
    #     contours, hierarchy = cv2.findContours(fg_mask_bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:]
    #     areas = [cv2.contourArea(c) for c in contours]
 
    #     # If there are no countours
    #     if len(areas) < 1:
 
    #         # Display the resulting frame
    #         cv2.imshow('frame',frame)
 
    #         # If "q" is pressed on the keyboard, 
    #         # exit this loop
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break
 
    #         # Go to the top of the while loop
    #         continue
 
    #     else:
    #         # Find the largest moving object in the image
    #         max_index = np.argmax(areas)
 
    #     # Draw the bounding box
    #     cnt = contours[max_index]
    #     x,y,w,h = cv2.boundingRect(cnt)
    #     cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)
 
    #     # Draw circle in the center of the bounding box
    #     x2 = x + int(w/2)
    #     y2 = y + int(h/2)
    #     cv2.circle(frame,(x2,y2),4,(0,255,0),-1)
 
    #     # Print the centroid coordinates (we'll use the center of the
    #     # bounding box) on the image
    #     text = "x: " + str(x2) + ", y: " + str(y2)
    #     cv2.putText(frame, text, (x2 - 10, y2 - 10),
    #         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
         
    #     # Display the resulting frame
    #     cv2.imshow('frame',frame)
 
        # If "q" is pressed on the keyboard, 
        # exit this loop
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
 
    # # Close down the video stream



# your existing centroid function
def getcentroid(cap,back_sub,kernel):
    """
    Dummy example returning the current centroid (x, y).
    Replace this with your actual detection logic.
    """
    ret, frame = cap.read()
 
        # Use every frame to calculate the foreground mask and update
        # the background
    fg_mask = back_sub.apply(frame)
 
        # Close dark gaps in foreground object using closing
    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)

    # Remove salt and pepper noise with a median filter
    fg_mask = cv2.medianBlur(fg_mask, 5) 
        
    # Threshold the image to make it either black or white
    _, fg_mask = cv2.threshold(fg_mask,127,255,cv2.THRESH_BINARY)

    # Find the index of the largest contour and draw bounding box
    fg_mask_bb = fg_mask
    contours, hierarchy = cv2.findContours(fg_mask_bb,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:]
    areas = [cv2.contourArea(c) for c in contours]

    # If there are no countours
    if len(areas) < 1:

        # Display the resulting frame
        return (0,0)

    else:
        # Find the largest moving object in the image
        max_index = np.argmax(areas)

    # Draw the bounding box
    cnt = contours[max_index]
    x,y,w,h = cv2.boundingRect(cnt)
    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)

    # Draw circle in the center of the bounding box
    x2 = x + int(w/2)
    y2 = y + int(h/2)
    cv2.circle(frame,(x2,y2),4,(0,255,0),-1)

    # Print the centroid coordinates (we'll use the center of the
    # bounding box) on the image
    text = "x: " + str(x2) + ", y: " + str(y2)
    cv2.putText(frame, text, (x2 - 10, y2 - 10),
    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
    # Display the resulting frame
    cv2.imshow('frame', frame)
    cv2.waitKey(1)


    # import random
    # # For testing, produce a simulated centroid moving left/right
    # x = random.uniform(0, 640)
    # y = random.uniform(0, 480)
    return (x2, y2)


class CentroidPublisher(Node):
    def __init__(self):
        super().__init__('centroid_follower')

        # publisher for the centroid
        self.pub_ = self.create_publisher(Point, '/object_centroid', 10)
        self.timer = self.create_timer(1, self.timer_callback)  # 10 Hz
        self.get_logger().info("Centroid publisher node started.")
        self.cap = cv2.VideoCapture(2)
 
    # Create the background subtractor object
    # Use the last 700 video frames to build the background
        self.back_sub = cv2.createBackgroundSubtractorMOG2(history=700, 
        varThreshold=25, detectShadows=True)
 
    # Create kernel for morphological operation
    # You can tweak the dimensions of the kernel
    # e.g. instead of 20,20 you can try 30,30.
        self.kernel = np.ones((20,20),np.uint8)

    def timer_callback(self):
        x, y = getcentroid(self.cap,self.back_sub,self.kernel)

        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0

        self.pub_.publish(msg)
        self.get_logger().info(f'Published centroid: ({x:.2f}, {y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = CentroidPublisher()
    rclpy.spin(node)
    # cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
