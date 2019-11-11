from abc import ABCMeta, abstractmethod

import numpy as np


class CollisionObject():
	"""
	Abstract class for a parametrically defined collision object.
	"""
	__metaclass__ = ABCMeta
	@abstractmethod
	def in_collision(self, target):
		"""
		Checks whether target point is in collision. Points at the boundary of
		the object are in collision.

		:returns: Boolean indicating target is in collision.
		"""
		pass


class CollisionBox(CollisionObject):
	"""
	N-dimensional box collision object.
	"""
	def __init__(self, location, half_lengths):
		"""
		:params location: coordinates of the center
		:params half_lengths: half-lengths of the rectangle along each axis
		"""
		self.location = np.asarray(location)
		self.half_lengths = np.asarray(half_lengths)
		self.ndim = self.location.shape[0]

	def in_collision(self, target):
		# FILL in your code here
		bottom_left_coordinate = self.location - self.half_lengths
		top_left_coordinate = self.location + self.half_lengths

		for bottom_left, top_right, target in zip(bottom_left_coordinate,top_left_coordinate,target):
			if not(target >= bottom_left and target <= top_right):
				return False;

		return True

class CollisionSphere(CollisionObject):
	"""
	N-dimensional sphere collision object.
	"""
	def __init__(self, location, radius):
		"""
		:params location: coordinates of the center
		:params radius: radius of the circle
		"""
		self.location = np.asarray(location)
		self.radius = radius

	def in_collision(self, target):
		# FILL in your code here
		dist = (self.location - target)**2
		dist = np.sum(dist)
		dist = np.sqrt(dist)

		if dist <= self.radius:
			return True

		else: 
			return False
