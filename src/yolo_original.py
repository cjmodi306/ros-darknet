#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from yolo_object_detector.msg import ObjectData
from yolo_object_detector.msg import ObjectIndices
import rospy
import os
import numpy as np

class ObjectDetector:
	def __init__(self):
		self.rate = rospy.Rate(10)
		self.PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
		self.CLASSES = os.path.join(self.PATH,'utils/classes.txt')
		self.WEIGHTS = os.path.join(self.PATH, 'weights/yolov3-tiny.weights')
		self.CONFIG = os.path.join(self.PATH, 'cfg/yolov3-tiny.cfg')
		with open(self.CLASSES, 'r') as f:
			self.class_names = [line.strip() for line in f.readlines()]
		self.COLORS = np.random.uniform(0,255, size=(len(self.class_names),3))
		self.scale = 0.00392
		self.model = cv2.dnn.readNet(self.WEIGHTS, self.CONFIG)
		self.model.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
		self.model.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
		self.objdata = ObjectData()
		self.objindices = ObjectIndices()
		self.pub_objdata = rospy.Publisher('ObjectData', ObjectData, queue_size=10)
		self.pub_objindices = rospy.Publisher('ObjectIndices', ObjectIndices, queue_size=10)
		self.detect()
	
	def get_output_layers(self,model):
		layer_names = self.model.getLayerNames()
		output_layers = [layer_names[i-1] for i in self.model.getUnconnectedOutLayers().flatten()]
		return output_layers
	
	def draw_bounding_boxes(self,img, class_id, confidence, x,y,x_w, y_h):
		self.label = str(self.class_names[class_id])
		self.color = self.COLORS[class_id]
		cv2.rectangle(img, (x, y), (x_w, y_h), self.color,2)
		cv2.putText(img, self.label, (x-10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

	def detect(self):
		rospy.Subscriber('/camera/right/image_raw', Image, self.callback)

	def callback(self,data):
		self.img = CvBridge().imgmsg_to_cv2(data, 'bgr8')
		(self.height, self.width, self.channels) = self.img.shape
		self.blob = cv2.dnn.blobFromImage(self.img, self.scale, (416,416), (0,0,0), True, crop=False)
		self.model.setInput(self.blob)
		outs= self.model.forward(self.get_output_layers(self.model))
		self.img = self.img/255
		class_ids = []
		confidences = []
		boxes = []
		conf_threshold = 0.3
		nms_threshold = 0.4

		for out in outs:
			for detection in out:
				scores = detection[5:]
				self.class_id = np.argmax(scores)
				confidence = scores[self.class_id]
				if confidence > conf_threshold:
					center_x = int(detection[0]*self.width)
					center_y = int(detection[1]*self.height)
					self.w = int(detection[2]*self.width)
					self.h = int(detection[3]*self.height)
					self.x = center_x - self.w/2
					self.y = center_y - self.h/2
					class_ids.append(self.class_id)
					confidences.append(float(confidence))
					boxes.append([self.x, self.y, self.w, self.h])
		indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)	
		for i in indices:
			box = boxes[i]
			x = box[0]
			y = box[1]
			w = box[2]
			h = box[3]
			self.draw_bounding_boxes(self.img, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
			self.objdata.object = self.class_names[class_ids[i]]
			self.objdata.x = x
			self.objdata.y = y
			self.objdata.h = h
			self.objdata.w = w
		rospy.loginfo(self.objdata)
		#cv2.imshow("Right camera window", self.img)
		#cv2.waitKey(1)


if __name__=='__main__':
	rospy.init_node('opencv_object_detector_right', anonymous=True)
	od = ObjectDetector()
	rospy.spin()
