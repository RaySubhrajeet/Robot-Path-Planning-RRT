import numpy as np
from collision import CollisionBox, CollisionSphere
import sys

class RRT(object):
	"""
	Simple implementation of Rapidly-Exploring Random Trees (RRT)
	"""
	class Node():
		"""
		A node for a doubly-linked tree structure.
		"""
		def __init__(self, state, parent):
			"""
			:param state: np.array of a state in the search space.
			:param parent: parent Node object.
			"""
			self.state = np.asarray(state)
			self.parent = parent
			self.children = []

		def __iter__(self):
			"""
			Breadth-first iterator.
			"""
			nodelist = [self]
			while nodelist:
				node = nodelist.pop(0)
				nodelist.extend(node.children)
				yield node

		def __repr__(self):
			return 'Node({})'.format(', '.join(map(str, self.state)))

		def add_child(self, state):
			"""
			Adds a new child at the given state.

			:param statee: np.array of new child node's statee
			:returns: child Node object.
			"""
			child = RRT.Node(state=state, parent=self)
			self.children.append(child)
			return child


	def __init__(self,
				 start_state,
				 goal_state,
				 dim_ranges,
				 obstacles=[],
				 step_size=0.05,
				 max_iter=1000):
		"""
		:param start_state: Array-like representing the start state.
		:param goal_state: Array-like representing the goal state.
		:param dim_ranges: List of tuples representing the lower and upper
			bounds along each dimension of the search space.
		:param obstacles: List of CollisionObjects.
		:param step_size: Distance between nodes in the RRT.
		:param max_iter: Maximum number of iterations to run the RRT before
			failure.
		"""
		self.start = RRT.Node(start_state, None)
		self.goal = RRT.Node(goal_state, None)
		self.dim_ranges = dim_ranges
		self.obstacles = obstacles
		self.step_size = step_size
		self.max_iter = max_iter

		if (self.start.state.shape != self.goal.state.shape):
			raise AssertionError("Start and Goal states do not match dimension!")

	def build(self):
		"""
		Build an RRT.

		In each step of the RRT:
			1. Sample a random point.
			2. Find its nearest neighbor.
			3. Attempt to create a new node in the direction of sample from its
				nearest neighbor.
			4. If we have created a new node, check for completion.

		Once the RRT is complete, add the goal node to the RRT and build a path
		from start to goal.

		:returns: A list of states that create a path from start to
			goal on success. On failure, returns None.
		"""
		for k in range(self.max_iter):
			# FILL in your code here
			sample=self._get_random_sample()
			nearest_node=self._get_nearest_neighbor(sample)
			new_node=self._extend_sample(sample,nearest_node)

			if new_node and self._check_for_completion(new_node):
				# FILL in your code here
				path=self._trace_path_from_start()
				
				return path

		print("Failed to find path from {0} to {1} after {2} iterations!".format(
			self.start.state, self.goal.state, self.max_iter))

	def _get_random_sample(self):
		"""
		Uniformly samples the search space.

		:returns: A vector representing a randomly sampled point in the search
			space.
		"""
		# FILL in your code here
		sample=[]
		for dim in self.dim_ranges:
			sample.append(np.random.uniform(dim[0],dim[1]))

		return sample


	def _get_nearest_neighbor(self, sample):
		"""
		Finds the closest node to the given sample in the search space,
		excluding the goal node.

		:param sample: The target point to find the closest neighbor to.
		:returns: A Node object for the closest neighbor.
		"""
		# FILL in your code here

		
		nearestnode=self.start
		nodes=self.start.__iter__()
		returndNodes=next(nodes,None)
		for n in returndNodes:			
			print(n.state)
			print("............................................................................")
		mindistance=sys.float_info.max
		for node in returndNodes:
			currdistance=self.get_distance(sample,node.state) 
			if currdistance < mindistance:
				mindistance=currdistance
				nearestnode=node
			print(nearestnode.state)
			print(mindistance)
			print("............................................................................")


		return nearestnode

	
	def get_distance(self,A, B):
		"""
		Returns Euclidean distance between two vectors
		"""

		dist = (A - B)**2
		dist = np.sum(dist)
		dist = np.sqrt(dist)

		return dist

	def _extend_sample(self, sample, neighbor):
		"""
		Adds a new node to the RRT between neighbor and sample, at a distance
		step_size away from neighbor. The new node is only created if it will
		not collide with any of the collision objects (see
		RRT._check_for_collision)

		:param sample: target point
		:param neighbor: closest existing node to sample
		:returns: The new Node object. On failure (collision), returns None.
		"""
		# FILL in your code here
		line_vector=np.subtract(sample,neighbor.state)   
		val=np.linalg.norm(line_vector)
		unit_vector=[linevec/val for linevec in line_vector]
		stepsizevector=[]
		for vec in unit_vector:
			stepsizevector.append(vec *self.step_size)
		
		newnode = [neighbor.state[i] + stepsizevector[i] for i in range(len(stepsizevector))] 

		if self._check_for_collision(newnode.state):
			return None

		else:
			child=neighbor.add_child(newnode)
			return child



	def _check_for_completion(self, node):
		"""
		Check whether node is within self.step_size distance of the goal.

		:param node: The target Node
		:returns: Boolean indicating node is close enough for completion.
		"""
		# FILL in your code here
		if self.get_distance(node.state,self.goal.state)<= self.step_size:
			return True
		else:
			return False

	def _trace_path_from_start(self, node=None):
		"""
		Traces a path from start to node, if provided, or the goal otherwise.

		:param node: The target Node at the end of the path. Defaults to
			self.goal
		:returns: A list of states (not Nodes!) beginning at the start state and
			ending at the goal state.
		"""
		# FILL in your code here
		nodes=self.start.__iter__()
		returnedNodes=next(nodes,None)
		path=[]
		is_goal_reached=True
		for nd in returnedNodes:
			path.extend(nd.state)
			for l1,l2 in zip(node.state,nd.state):
				if l1 != l2:
					is_goal_reached=False
			if is_goal_reached:
				return path

		return None

	def _check_for_collision(self, sample):
		"""
		Checks if a sample point is in collision with any collision object.

		:returns: A boolean value indicating that sample is in collision.
		"""
		# FILL in your code here
		return False
