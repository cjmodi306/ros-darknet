#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from yolo_object_detector.msg import ObjectData
from yolo_object_detector.msg import MultiObjectData
from yolo_object_detector.msg import coordinates
import rospy
from sensor_msgs.msg import NavSatFix
import os
import numpy as np
import sys
import time

class ObjectDetector:
	def __init__(self):
		self.rate = rospy.Rate(10)
		self.mod = MultiObjectData()
		self.pub = rospy.Publisher('MultiObjectData', MultiObjectData, queue_size=10)
		self.detect()
		
	def detect(self):
		rospy.Subscriber('/sensor_msgs/NavSatFix', NavSatFix, self.callback)

	def callback(self,data):
		rospy.loginfo(data.latitude, data.longitude)
		
if __name__=='__main__':
	rospy.init_node('opencv_object_detector_right', anonymous=True)
	rospy.loginfo('Getting GPS data...')
	#od = ObjectDetector()
	rospy.spin()
