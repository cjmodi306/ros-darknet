#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from yolo_object_detector.msg import MultiObjectData
from yolo_object_detector.msg import MultiStereoData
import rospy
import os
import numpy as np
import matplotlib.pyplot as plt
import sys

class YoloStereo:
	def __init__(self):
		self.obj = None
		self.x = 0
		self.y = 0
		self.h = 0
		self.w = 0
		self.person = 0
		self.distance = 0
		self.COLORS = (0,0,255)
	
	def draw_bounding_boxes(self,img, text, x,y,x_w, y_h):
		cv2.rectangle(img, (x, y), (x_w, y_h), self.COLORS,2)
		cv2.putText(img, text, (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 2,self.COLORS, 3)
		
	def stereo_callback(self,data):
		 distance_list = [data.object_distance[0].distance for i in range(len(data.object_distance)) if data.object_distance[i].object == self.obj]
		 if len(distance_list) > 0:
		 	self.distance = distance_list[0]
		 
	def getCoordinates(self,data):
		for i in range(len(data.objects)):
			self.obj = data.objects[i].object
			self.x = data.objects[i].x
			self.y = data.objects[i].y
			self.h = data.objects[i].h
			self.w = data.objects[i].w
	
	def displayResult(self, data):
		try:	
			image = CvBridge().imgmsg_to_cv2(data, 'bgr8')	
			text = self.obj +": " + str(round(self.distance, 2)) + "m"
			self.draw_bounding_boxes(image, text , round(self.x), round(self.y), round(self.x+self.w), round(self.y+self.h))
			#cv2.imshow("Main window", image)
			#cv2.waitKey(1)
		except:
			pass

	def image2Signal(self, data):
		x = int(self.x)
		y = int(self.y)
		x_w = int(self.x + self.w)
		y_h = int(self.y + self.h)
		dispImage = CvBridge().imgmsg_to_cv2(data.image, desired_encoding='32FC1')
		region_of_interest = dispImage[y:y_h, x:x_w]
		mask = np.ones((region_of_interest.shape))*-1
		boolean_mask = np.array(region_of_interest != mask)
		masked_img = np.ma.MaskedArray(region_of_interest, mask=~boolean_mask)
		masked_img = [i for i in masked_img.compressed()]
		if len(masked_img) > 0:
			print(self.obj)
			plt.plot(masked_img)
			plt.show()
			rospy.sleep(5)
			sys.exit()
			
if __name__=='__main__':
	rospy.init_node('opencv_object_detector_right', anonymous=True)
	ys = YoloStereo()
	rospy.Subscriber('/MultiObjectData', MultiObjectData, ys.getCoordinates)
	rospy.Subscriber('/StereoData', MultiStereoData, ys.stereo_callback)
	rospy.Subscriber('/camera/right/image_raw', Image, ys.displayResult)
	#rospy.Subscriber('/camera/disparity', DisparityImage, ys.image2Signal)
	rospy.spin()
