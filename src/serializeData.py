#!/usr/bin/env python3

import rospy
from yolo_object_detector.msg import MultiStereoData
import socket
import json		
from multiprocessing import Process, Queue
import numpy as np
from threading import Thread
from multiprocessing.connection import Listener, Client
from sensor_msgs.msg import Image

class Serialization:
	serializedData = None
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	bufferSize = 1024
	sock.bind(('', 8082))
	address = None
	
	def __init__(self):
		self.msd = MultiStereoData()
		self.img = None
		
	def start(self):
		'''
		Method to read topics that include the detected objects
		and their corresponding distances.
		'''
		rospy.Subscriber('/StereoData', MultiStereoData, self.stereoCallback)
		#rospy.spin()

	def stereoCallback(self, data):
		'''
		Callback Method to read to unpack the data from the topics and
		send them to convertToDict method to convert the class data into dictionaries.
		'''
		stereoObjectList = []
		stereoObjectList += [[i.object, round(i.distance,2)] for i in data.object_distance]
		stereoDict = self.convertToDict(np.array(stereoObjectList))
		self.clientConnect(stereoDict)
	
	def cam_callback(self,data):
		self.img = CvBridge().imgmsg_to_cv2(data, 'bgr8')
		
	def draw_bounding_boxes(self,img, text, x,y,x_w, y_h):
		cv2.rectangle(img, (x, y), (x_w, y_h), self.COLORS,2)
		cv2.putText(img, text, (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 2,self.COLORS, 3)

	def convertToDict(self,arr):
		'''
		Method to convert the unpacked topics into dictionaries.
		'''
		set_arr = set(arr[:,0])
		stereoDict = {}
		for elem in set_arr:
			num = 1
			count = (arr==elem).sum()
			indices = np.where(arr==elem)[0]
			for idx in indices:
				if count > 1:
					stereoDict[elem + str(num)] = arr[idx, 1]
					num += 1
				else:
					stereoDict[elem] = arr[idx,1]
		return stereoDict
		
	def publishData(self, data):
		'''
		Method to serialize dictionaries and transfer them via socket over UDP.
		'''
		self.serializedData = json.dumps(data).encode('utf8')
		self.clientConnect()
		rospy.loginfo(self.serializedData)
	
	
	def clientConnect(self, data):
		bytesAddressPair = self.sock.recvfrom(self.bufferSize)
		address = bytesAddressPair[1]
		serializedData = json.dumps(data).encode('utf8')
		if address:
			self.sock.sendto(serializedData, address)

		
if __name__=='__main__':
	rospy.init_node('UDP_Connection', anonymous=True)
	s = Serialization()
	proc = Thread(target=s.start)
	proc.start()
	rospy.spin()
