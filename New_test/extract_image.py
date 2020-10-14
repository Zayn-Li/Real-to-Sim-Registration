# coding:utf-8
#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys

rgb_path = './color_image/'

class ImageCreator():
	def __init__(self):
	    self.bridge = CvBridge()
	    with rosbag.Bag('./data_1013/rosbag/2020-10-13-22-05-39.bag', 'r') as bag:  
		for topic,msg,t in bag.read_messages():
		    if topic == "/camera/color_image": 
		            try:
		                cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
		            except CvBridgeError as e:
		                print e
		            timestr = "%.15f" %  msg.header.stamp.to_sec()
		            image_name = timestr+ ".png" 
		            cv2.imwrite(rgb_path + image_name, cv_image) 

if __name__ == '__main__':
	try:
	    image_creator = ImageCreator()
	except rospy.ROSInterruptException:
	    pass
