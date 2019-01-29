import numpy as np
from scipy.sparse.csgraph import dijkstra

class Maze:
	def __init__(self, width=16, height=16):
		self.width = width
		self.height = height

		self.adj_matrix = np.ones([self.width * self.height, self.width * self.height])
		self.disconnect_non_adjacent()

	def disconnect_non_adjacent(self):
		for c1_index in xrange(self.width * self.height):
			for c2_index in xrange(self.width * self.height):
				c1 = self.find_x_y(c1_index)
				c2 = self.find_x_y(c2_index)
				self.adj_matrix[c1, c2] = ((abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])) == 1)
		

	def get_connected(self, c1, c2):
		"""Check whether a wall exists between adjacent cells
		
		Args:
		    c1 (tuple): x,y coordinate of cell1
		    c2 (tuple): x,y coodrinate of cell2
		
		Returns:
		    float: probability of wall between those two cells
		"""
		assert (abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])) == 1, 'Cells must be adjacent.'
		c1_cell_number = self.find_adjacency_cell_number(c1)
		c2_cell_number = self.find_adjacency_cell_number(c2)

		return self.adj_matrix[max(c1_cell_number,c2_cell_number),min(c1_cell_number, c2_cell_number)]

	def set_connected(self, c1, c2, v):
		"""Set whether a wall exists between adjacent cells
		
		Args:
		    c1 (tuple): x,y coordinate of cell1
		    c2 (tuple): x,y coodrinate of cell2
		    v  (float): probability of a wall between the cells
		"""
		assert (abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])) == 1, 'Cells must be adjacent.'
		c1_cell_number = self.find_adjacency_cell_number(c1)
		c2_cell_number = self.find_adjacency_cell_number(c2)

		self.adj_matrix[max(c1_cell_number,c2_cell_number),min(c1_cell_number, c2_cell_number)] = v

	# TODO: Fix me! 
	# def __str__(self):
	# 	output = ''
	# 	for row in range(self.height + 1):
	# 		horizontal = ['+---' if w > 0 else '+   ' for w in self.h_walls[row,:]]
	# 		output += (''.join(horizontal) + '+\n')

	# 		if row < self.height:
	# 			vertical = ['|   ' if w > 0 else '    ' for w in self.v_walls[row,:]]
	# 			output += (''.join(vertical)[:-3] + '\n')

		# return output

	def find_route(self, c1, c2, threshold = 0):
		r = dijkstra(self.adj_matrix, directed = False, unweighted = True, return_predecessors = True)
		c1_index = self.find_adjacency_cell_number(c1)
		c2_index = self.find_adjacency_cell_number(c2)
		path = []
		print(r[1])
		self.unwind_predecessor_matrix(r[1], c1_index, c2_index, path)
		return path

	def unwind_predecessor_matrix(self, r, c1, c2, path):
		intermediate_cell = r[c1, c2]
		if intermediate_cell < 0:
			return 
		path.append(intermediate_cell)
		print("intermediate: %s c1: %s c2: %s path: %s" % (intermediate_cell, c1, c2, path))
		self.unwind_predecessor_matrix(r, c1, intermediate_cell, path)

	def find_adjacency_cell_number(self, c1):
		return self.width * c1[1] + c1[0]

	def find_x_y(self, n):
		return np.array([n % self.width, np.floor(n / self.width)], dtype=int)

	def build_adjacency(self):
		self.v_walls
		self.h_walls


if __name__ == '__main__':
	m = Maze(2,2)
	# add some walls
	# m.h_walls[1,1] = 1
	# m.h_walls[2,3] = 1
	# m.v_walls[2,3] = 1
	# m.set_connected([1,1],[0,1], 1)
	# m.set_connected([1,2],[1,1], 1)
	# see what it looks like
	print(m)