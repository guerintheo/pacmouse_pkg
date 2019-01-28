import numpy as np

# TODO(izzy): how do we want to index the maze? Right now I've got the (0,0)
# cell in the top left corner

# NOTE(izzy): we might decide to use this class with probailities on the walls
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