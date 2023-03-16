#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Code adapted from flip_image.py and lecture code

img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True

	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('detect_ball', anonymous = True)
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	img_pub = rospy.Publisher('/Ball_2D', Image, queue_size = 1)
	
	crop_img = np.zeros((720, 1280, 3), dtype = "uint8")
	crop_img = cv2.rectangle(crop_img, (100, 100), (620, 1180), (255, 255, 255))
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
		
			hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
			# define the upper and lower ranges
			lower_yellow_hsv = np.array([20,5,1])
			upper_yellow_hsv = np.array([60,255,255])
			# filter the image 
			yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
			
			cropped_hsv = cv2.bitwise_and(yellow_mask, crop_img)
			# convert it to ros msg and publish itcd
			img_msg = CvBridge().cv2_to_imgmsg(cropped_hsv, encoding="mono8")
			# publish the image
			img_pub.publish(img_msg)
			
		# pause until the next iteration			
		rate.sleep()
