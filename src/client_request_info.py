#!/usr/bin/env python

import rospy
import tf2_ros
from threading import Lock
import tf.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from py_tf2_web.srv import *
from py_tf2_web.msg import *
from tf_pair import TFPair


class ClientRequestInfo:

	def __init__(self, clientIDCount, topicName, req, tfBufferMutex, tfBuffer):

		self.tfSubscriptions=[]
		self.tfBufferMutex = tfBufferMutex
		self.tfBuffer = tfBuffer
		self.clientID = clientIDCount
		clientIDCount += 1
		self.pub = rospy.Publisher(topicName, TFArray, queue_size=10)
		#print(req.source_frames);
		self.setSubscriptions(req.source_frames, req.target_frame, req.parent, req.angular_thres, req.trans_thres)
		self.unsubTimeout = req.timeout
		#self.unsubTimer = rospy.Timer(self.unsubTimeout, self.unadvertiseCB , True)
		duration=rospy.Duration.from_sec(1.0 / req.rate )
		rospy.sleep(2.)
		self.timer=rospy.Timer(duration, self.processRequest)
		self.isActive = True
		self.twist = Twist()
		self.twist.linear.x = 0.0
		self.twist.angular.z = 0.0
		self.velocityTopic = '/p3dx/cmd_vel'
		self.arrayMsg = None
		self.customTF = None
		

	def setSubscriptions(self, source_frames, target_frame, parent, angular_thres, trans_thres):
		
		requestSize = len(source_frames)
		print(source_frames);
		print(parent);
		for i in range (0, requestSize):
			tfPair = TFPair()
			sourceFrame = self.cleanTfFrame(source_frames[i])
			if parent[i] == -1 :
				targetFrame = self.cleanTfFrame(target_frame)
			else :
				targetFrame = self.cleanTfFrame(source_frames[parent[i]])

			
			tfPair._sourceFrame = sourceFrame
			tfPair._targetFrame = targetFrame
			tfPair._angularThres = float("{0:.3f}".format(angular_thres))
			tfPair._transThres = float("{0:.3f}".format(trans_thres))
			tfPair._index = i
			self.tfSubscriptions.append(tfPair)

	def cleanTfFrame(self,frameID):
		if frameID.startswith('/') :
			return frameID[1:]
		return frameID

	def unadvertiseCB (self, event):
		self.pub.unregister()
		self.unsubTimer.shutdown()
		self.timer.shutdown()
		self.isActive = False

	def callback(self, twist):
		if self.twist.linear.x != twist.linear.x or self.twist.angular.z != twist.angular.z :
			self.twist.linear.x = twist.linear.x
			self.twist.angular.z = twist.angular.z
			if len(self.arrayMsg.transforms) > 0 :
				self.pub.publish(self.arrayMsg)
				#print("Request %d: TFs published:", requestInfo.clientID)
			else:
				#print("Request %d: No TF frame update needed:", requestInfo.clientID)
				pass

	def processRequest(self, event):
		if self.pub.get_num_connections == 0:
			self.unsubTimer.run()
		else :
			#self.unsubTimer.shutdown()
			self.arrayMsg = TFArray()
			#updateSubscriptions begin
			subscriptionSize = len(self.tfSubscriptions)

			for i in range(0, subscriptionSize):
				it = self.tfSubscriptions[i]
				#print(it._index);
				self.tfBufferMutex.acquire()
				transform = self.tfBuffer.lookup_transform(it._targetFrame,
												it._sourceFrame,
												rospy.Time())

				index = it._index
				self.tfBufferMutex.release()
						
				if not it._isOkay :
					it._isOkay = True
					print ("Transform from %s to %s is working again",
							 it._sourceFrame,
							 it._targetFrame)

				it.updateTransform(transform)
				if it.updateNeeded():
					print(it._targetFrame)
					print(it._sourceFrame)
					print(it._index)
					#print(it._index);
					transform.header.stamp = rospy.Time().now()
			 		transform.header.frame_id = it._targetFrame
			 		transform.child_frame_id = it._sourceFrame
			 		it.transmissionTriggered()
			 		self.customTF = CustomTF()
			 		self.customTF.transf = transform
			 		self.customTF.index = index
			 		self.arrayMsg.transforms.append(self.customTF)
				#update subscriptions end
				#rospy.Subscriber(self.velocityTopic, geometry_msgs.msg.Twist, self.callback)
			if len(self.arrayMsg.transforms) > 0 :
				self.pub.publish(self.arrayMsg)