#!/usr/bin/env python

import tf.msg
import geometry_msgs.msg 
from math import sqrt, acos




class TFPair:

	def __init__(self):
		self._isOkay = True
		self._sourceFrame = None
		self._targetFrame = None
		self._angularThres = 0.000
		self._transThres = 0.000
		self._updated = False
		self._firstTransmission = True
		self._tfTransmitted = geometry_msgs.msg.Transform()
		self._tfReceived = None
		self._index = None

	def updated(self, updated):
		self._updated = updated
		self._updated = True

	def updateTransform(self, update):

		self._tfReceived = geometry_msgs.msg.Transform()
		self._tfReceived.translation.x = update.transform.translation.x
		self._tfReceived.translation.y = update.transform.translation.y
		self._tfReceived.translation.z = update.transform.translation.z
		self._tfReceived.rotation.x = update.transform.rotation.x
		self._tfReceived.rotation.y = update.transform.rotation.y
		self._tfReceived.rotation.z = update.transform.rotation.z
		self._tfReceived.rotation.w = update.transform.rotation.w
		self._updated = True

	def updateNeeded(self):
		result = False
		if self._firstTransmission :
			result = True
			self._firstTransmission = False
		else :
			if self._updated :
				if self.distance(self._tfReceived.translation, self._tfTransmitted.translation) > self._transThres or self.angle(self._tfReceived.rotation, self._tfTransmitted.rotation) >self._angularThres :
					result = True
					self._firstTransmission = False
		_updated = False
		return result

	def transmissionTriggered(self):
		self._tfTransmitted = self._tfReceived

	def distance(self, v1, v2):
		v = geometry_msgs.msg.Vector3(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z)
		return sqrt(self.dotVector(v, v))

	def angle (self, q1, q2):
		s = float("{0:.3f}".format(sqrt(self.dotQuaterion(q1, q1) * self.dotQuaterion(q2,q2))))
		if s != 0 :
			t =float("{0:.3f}".format(self.dotQuaterion(q1, q2) / s))
			if t > 1:
				t = 1.000
			if t < -1 :
				t = -1.000
			ret = acos(t)
			return ret
		else:
			return -1

	def dotVector(self, v1, v2):
		return (float("{0:.3f}".format(v1.x * v2.x + v1.y * v2.y + v1.z * v2.z)))

	def dotQuaterion(self, q1, q2):
		return (float("{0:.3f}".format(q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w)))