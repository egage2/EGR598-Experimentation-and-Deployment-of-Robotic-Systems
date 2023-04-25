#EGR598:Experimentation and Deployment of Robotic Systems Spring 2023
#Final Project - Color Dependent Blob follower/tracker with filtered data
#Image Subscriber and Blob publisher
#Ethan Gage
#Akshay Fulzele
#Daniel Espinoza-Pena

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import time

class ImageSubscriber(Node):
  def __init__(self):
    self.past = time()
    super().__init__('subscriber')
    self.subscription = self.create_subscription(Image,'/color/preview/image', self.listener_callback, 10)
    self.publisher_ = self.create_publisher(String, 'blobs', 10)
    
    self.br = CvBridge()
    
    self.im = cv2.imread("blobs.png", cv2.IMREAD_GRAYSCALE)
    
    #values for color masks
    self.lower_red = np.array([130, 100, 0 ])
    self.upper_red = np.array([180, 255, 240])
    self.lower_green = np.array([30, 70, 50 ])
    self.upper_green = np.array([90, 240, 230])
    self.lower_blue = np.array([94, 80, 2])
    self.upper_blue = np.array([126, 255, 255])
    
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
       
    #setup blob detector with color masks
    params = cv2.SimpleBlobDetector_Params()
    hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
    blue_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
    green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
    
    # Parameters for blob detector
    params.filterByArea = True
    params.minArea = 250
    params.maxArea = 10000
    params.minInertiaRatio = 0.05
    params.minConvexity = .60
    params.filterByCircularity = True
    params.minCircularity = 0.5

    params.filterByColor = True
    params.blobColor = 255
    params.filterByConvexity = False
    params.minConvexity = 0.2

    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    detector = cv2.SimpleBlobDetector_create(params)
 
    # Detect blobs.
    red_keypoints = detector.detect(red_mask)
    blue_keypoints = detector.detect(blue_mask)
    green_keypoints = detector.detect(green_mask)
    
    
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(current_frame, red_keypoints, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    text = "Red Blobs: " + str(len(red_keypoints))
    cv2.putText(blobs, text, (10, 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    #Create Message Publisher
    msg = String()
    if len(red_keypoints) > 0:
        blobs = cv2.drawKeypoints(current_frame, red_keypoints, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        text = "Red Blobs: " + str(len(red_keypoints))
        cv2.putText(blobs, text, (10, 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)        
        msg.data = msg.data + "red\n"
        msg.data = msg.data + str(red_keypoints[0].pt[0]) + "\n"            #x point of first blob (largest red)
        msg.data = msg.data + str(red_keypoints[0].pt[1]) + "\n" #y point of first blob (largest red)
        msg.data = msg.data + str(red_keypoints[0].size) + "\n" #size of blob
        
    if len(blue_keypoints) > 0:
        blobs = cv2.drawKeypoints(current_frame, blue_keypoints, blank, (255, 0, 0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        text = "Blue Blobs: " + str(len(blue_keypoints))
        cv2.putText(blobs, text, (10, 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        msg.data = msg.data + "blue\n"
        msg.data = msg.data + str(blue_keypoints[0].pt[0]) + "\n"            #x point of first blob (largest blue)
        msg.data = msg.data + str(blue_keypoints[0].pt[1]) + "\n" #y point of first blob (largest green)
        msg.data = msg.data + str(blue_keypoints[0].size) + "\n" #size of blob
       
    if len(green_keypoints) > 0:
        blobs = cv2.drawKeypoints(current_frame, green_keypoints, blank, (0, 255, 0),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
        text = "Green Blobs: " + str(len(green_keypoints))
        cv2.putText(blobs, text, (10, 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print(green_keypoints[0].size)
        msg.data = msg.data + "green\n"
        msg.data = msg.data + str(green_keypoints[0].pt[0]) + "\n"            #x point of first blob (largest red)
        msg.data = msg.data + str(green_keypoints[0].pt[1]) + "\n" #y point of first blob (largest red)
        msg.data = msg.data + str(green_keypoints[0].size) + "\n" #size of blob
    self.publisher_.publish(msg)
   
    cv2.imshow('Red mask', red_mask)
    cv2.imshow('Green mask', green_mask)
    cv2.imshow('Blue mask', blue_mask)
    cv2.imshow("Circular Blobs Only", blobs)
    
    
    
    cv2.waitKey(10)

def main(args=None):
  previous = time()
  delta = 0
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()