#!/usr/bin/env python


import rospy
import tf2_ros

import collections

from threading import Lock

import tf.msg
import geometry_msgs.msg

from py_tf2_web.srv import *
from py_tf2_web.msg import *

from tf_pair import TFPair
from client_request_info import ClientRequestInfo




class tfWebRepublisher:

	def __init__(self):
		self.tfBuffer = tf2_ros.Buffer()
		self.tf_listener_ = tf2_ros.TransformListener(self.tfBuffer)
		self.clientIDCount = 0
		self.requestsList = []
		self.requestsMutex = Lock()
		self.tfBufferMutex = Lock()

	def requestCB(self, req):
		topicname = 'tf_repub_new_' + str(self.clientIDCount)
		requestInfo = ClientRequestInfo(self.clientIDCount, topicname, req, self.tfBufferMutex, self.tfBuffer)
		self.requestsMutex.acquire()
		self.requestsList.append(requestInfo)
		self.requestsMutex.release()
		return RepublishNewTFsResponse(requestInfo.pub.name)
		
	def requestCB_server(self):
		
		s=rospy.Service('republish_tfs', RepublishNewTFs, self.requestCB)
		s.spin()


if __name__=='__main__':

	rospy.init_node('py_tf2_web')
	tfWebRepublisher = tfWebRepublisher()
	#service
	tfWebRepublisher.requestCB_server()
	
	#tf2 listener
	
	
	
