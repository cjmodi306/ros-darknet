#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from stereo_msgs.msg import DisparityImage
from yolo_object_detector.msg import StereoData
from yolo_object_detector.msg import MultiStereoData
from yolo_object_detector.msg import MultiObjectData
from sensor_msgs.msg import Image
import warnings
import rospy
import os
import numpy as np
import imutils
import websockets
from multiprocessing import Process, Queue
from scipy import ndimage
import matplotlib.pyplot as plt
	
class StereoCamera:
	def __init__(self):
		'''
		Disparity Range: 300
		Baseline: 34 cm
		'''
		self.pub = rospy.Publisher('StereoData', MultiStereoData, queue_size=10)
		self.dispImage = None
		self.focus = 0
		self.baseline = 0
		self.x = 0
		self.y = 0
		self.w = 0
		self.h = 0
		self.x_w = 0
		self.y_h = 0
		self.distance = 0
		self.multiple_stereo_data = MultiStereoData()
		self.region_of_interest = None

	def yolo_callback(self, data):
		'''
		Callback method to read the topics from YOLO and pass them to MultiProcessCallback
		to unpack and use the data to calculate distances.
		'''
		queue = Queue()
		proc = [Process(target=self.MultiProcessCallback, args=(data.objects[i],queue)) for i in range(len(data.objects))]
		[p.start() for p in proc]
		for p in proc:
			self.stereo_data = StereoData()
			self.stereo_data.object, self.stereo_data.distance = queue.get()
			self.multiple_stereo_data.object_distance += [self.stereo_data]
		rospy.loginfo(self.multiple_stereo_data.object_distance)
		self.pub.publish(self.multiple_stereo_data)
		
		for i in range(len(data.objects)):
			x = data.objects[i].x + 0
			y = data.objects[i].y
			w = data.objects[i].w
			h = data.objects[i].h
			self.showImage(int(x),int(y),int(x+w), int(y+h))	
		self.multiple_stereo_data.object_distance.clear()
	
	def MultiProcessCallback(self, data, queue):
		'''
		Unpack 
		'''
		object_type = data.object
		x = int(data.x) + 0
		y = int(data.y)
		w = int(data.w)
		h = int(data.h)
		distance = self.measureDistance(object_type, x,y,w,h)
		queue.put([object_type, distance])
				
	def disparity_callback(self,data):
		self.dispImage = CvBridge().imgmsg_to_cv2(data.image, desired_encoding='32FC1')
		self.focus = data.f
		self.baseline = data.T	
	
	def maskImage(self,x,y,x_w,y_h):
		self.region_of_interest = self.dispImage[y:y_h, x:x_w]
		mask = np.ones((self.region_of_interest.shape))*-1
		boolean_mask = np.array(self.region_of_interest != mask)
		masked_img = np.ma.MaskedArray(self.region_of_interest, mask=~boolean_mask)
		disparity = [np.mean([i for i in masked_img.compressed() if i>0])]
		return disparity[0]
		
	def measureDistance(self,object_type, x,y,w,h):
		'''
		Method to calculate distance of objects based on their coordinates and disparity values.
		'''
		distance = (self.focus*self.baseline)/self.maskImage(x,y,x+w,y+h)
		#print(self.focus, self.baseline)
		if ~np.isnan(distance):
			return round(distance, 2)
		else:
			return -1
			
	def imgCallback(self,data):
		self.img = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8')
	
	def showImage(self,x,y,x_w,y_h):
		cv2.rectangle(self.dispImage,(x,y),(x_w,y_h), (255,255,255),6)
		cv2.imshow("Disparity Image", self.dispImage)
		cv2.waitKey(1)			

if __name__=='__main__':
	rospy.init_node('opencv_object_detector_right', anonymous=True)
	warnings.simplefilter('ignore')
	sc = StereoCamera()
	rospy.Subscriber('/camera/disparity', DisparityImage, sc.disparity_callback)
	rospy.Subscriber('/camera/right/image_raw', Image, sc.imgCallback)
	rospy.Subscriber('/MultiObjectData', MultiObjectData, sc.yolo_callback)
	rospy.spin()
