import numpy as np

class Maze:
	def __init__(self, width=16, height=16):
		self.width = width
		self.height = height

		# store the north-south walls in v_walls and east-west walls in h_walls
		self.h_walls = np.zeros([self.height + 1, self.width])
		self.v_walls = np.zeros([self.height, self.width + 1])

		self.add_perimeter_walls()

	def add_perimeter_walls(self):
		self.v_walls[:, [0,-1]] = 1
		self.h_walls[[0,-1], :] = 1

	def get_connected(self, c1, c2):
		"""Check whether a wall exists between adjacent cells
		
		Args:
		    c1 (tuple): x,y coordinate of cell1
		    c2 (tuple): x,y coodrinate of cell2
		
		Returns:
		    float: probability of wall between those two cells
		"""
		assert (abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])) == 1, 'Cells must be adjacent.'
		if c1[0] == c2[0]:
			return self.h_walls[max(c1[1], c2[1]), c1[0]]
		else:
			return self.v_walls[c1[1], max(c1[0], c2[0])]

	def set_connected(self, c1, c2, v):
		"""Set whether a wall exists between adjacent cells
		
		Args:
		    c1 (tuple): x,y coordinate of cell1
		    c2 (tuple): x,y coodrinate of cell2
		    v  (float): probability of a wall between the cells
		"""
		assert (abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])) == 1, 'Cells must be adjacent.'
		if c1[0] == c2[0]:
			self.h_walls[max(c1[1], c2[1]), c1[0]] = v
		else:
			self.v_walls[c1[1], max(c1[0], c2[0])] = v

	def __str__(self):
		output = ''
		for row in range(self.height + 1):
			horizontal = ['+---' if w > 0 else '+   ' for w in self.h_walls[row,:]]
			output += (''.join(horizontal) + '+\n')

			if row < self.height:
				vertical = ['|   ' if w > 0 else '    ' for w in self.v_walls[row,:]]
				output += (''.join(vertical)[:-3] + '\n')

		return output


if __name__ == '__main__':
	m = Maze(5,4)
	# add some walls
	m.h_walls[1,1] = 1
	m.h_walls[2,3] = 1
	m.v_walls[2,3] = 1
	# see what it looks like
	print(m)