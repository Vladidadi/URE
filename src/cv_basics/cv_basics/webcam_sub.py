# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np


# def centroid(current_frame):    # Create a VideoCapture object
#     # # cap = cv2.VideoCapture(0)
 
#     # # Create the background subtractor object
#     # # Use the last 700 video frames to build the background
#     back_sub = cv2.createBackgroundSubtractorMOG2(history=700, 
#     varThreshold=25, detectShadows=True)
 
#     # # Create kernel for morphological operation
    # # You can tweak the dimensions of the kernel
    # # e.g. instead of 20,20 you can try 30,30.
 
    # while(True):
 
    #     # Capture frame-by-frame
    #     # This method returns True/False as well
    #     # as the video frame.
    #     ret, frame = cap.read()
#     kernel = np.ones((20,20),np.uint8)

#     # Use every frame to calculate the foreground mask and update
#     # the background
#     fg_mask = back_sub.apply(current_frame)

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

#     # If "q" is pressed on the keyboard, 
#     # exit this loop
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Close down the video stream
# cap.release()
# cv2.destroyAllWindows()

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    centroid(current_frame)
    # Display image
    cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
