import numpy as np
from scipy.sparse.csgraph import dijkstra

class Maze:

	"""Stores a representation of the maze.

	The maze is indexed by [x,y] (increasing to the right, increasing down)
	coordinates with the 0,0 cell in the top left corner.

	The maze is stored as an adjacency matrix. A value of 1 in the matrix
	indicates that the cells are connected. A value of zero indicates that
	the cells are disconnected.
	
	Attributes:
	    adj_matrix (TYPE): Description
	    height (int): The width of the Maze
	    width (int): The height of the Maze
	"""
	
	def __init__(self, width=16, height=16, filename=None):
		if filename is not None:
			self.load(filename)
		else:
			self.width = width
			self.height = height

			self.adj_matrix = np.zeros([self.width * self.height, self.width * self.height])
			self.generate_random_maze()
			# self.connect_all()

	def connect_all(self):
		for x in range(self.width):
			for y in range(self.height):
				if x < self.width - 1:
					self.set_connected([x,y], [x+1, y], 1)
				if y < self.height - 1:
					self.set_connected([x,y], [x, y+1], 1)

	def save(self, name):
		# save as plaintext so we can store a header
		file_format_explanation = "Lines starting with '#' will be ignored. This map represents:"
		np.savetxt(name, self.adj_matrix, header='{},{}\n{}\n{}'.format(self.width, self.height, file_format_explanation, self.__str__()))

	def load(self, name):
		with open(name) as f:
			w, h = f.readline()[2:-1].split(',') # read the header
			self.width = int(w)
			self.height = int(h)
			self.adj_matrix = np.loadtxt(f)

	def get_connected(self, c1, c2):
		"""Check whether it is possible to pass between adjacent cells
		
		Args:
		    c1 (tuple): x,y coordinate of cell1
		    c2 (tuple): x,y coodrinate of cell2
		
		Returns:
		    float: probability it is possible to pass between these cells
		"""
		assert self.check_adjacent_xy(c1, c2), 'Cells must be adjacent.'
		c1_index = self.xy_to_index(c1)
		c2_index = self.xy_to_index(c2)

		return self.adj_matrix[max(c1_index, c2_index),min(c1_index, c2_index)]

	def set_connected(self, c1, c2, v):
		"""Set whether it is possible to pass between adjacent cells
		
		Args:
		    c1 (tuple): x,y coordinate of cell1
		    c2 (tuple): x,y coodrinate of cell2
		    v  (float): probability it is possible to pass between these cells
		"""
		assert self.check_adjacent_xy(c1, c2), 'Cells must be adjacent.'
		c1_index = self.xy_to_index(c1)
		c2_index = self.xy_to_index(c2)

		self.adj_matrix[max(c1_index,c2_index),min(c1_index, c2_index)] = v

	def solve(self, threshold=0):
		"""Use Dijkstras to find pairwise distances between all cells in the maze
		
		Args:
		    threshold (int, optional): how confident we must be to pass between two cells
		"""
		self.dists, self.predecessors = dijkstra(self.adj_matrix > threshold, directed=False,
									    unweighted=True, return_predecessors=True)

	def generate_random_maze(self, discrete=True):
		"""Generates a random maze given the class size and height. 

		Args: 
			discrete (boolean): Whether the generated maze should be discrete (0,1) or continous (0.1~0.8)
		"""
		visited = np.zeros([self.height, self.width])
		visited = np.pad(visited, 1, 'constant', constant_values=1)
		
		def dfs(self, current):
			visited[current] = 1
			y = current[0]
			x = current[1]
			frontier = [(y-1,x), (y+1,x), (y,x-1), (y,x+1)]
			np.random.shuffle(frontier)

			for next in frontier:
				if visited[next]: continue
				self.set_connected((next[1]-1,next[0]-1), (current[1]-1,current[0]-1), 1 if discrete else np.random())

				dfs(self, next)
		start = (1,1)
		dfs(self, start)

		# Add 2x2 square to the center of the maze
		if self.width > 3 and self.height > 3:
			x_middle , y_middle = (int(np.floor(self.width/2))-1, int(np.floor(self.height/2))-1)
			self.set_connected((x_middle, y_middle),(x_middle + 1, y_middle),1)
			self.set_connected((x_middle, y_middle),(x_middle, y_middle + 1),1)
			self.set_connected((x_middle + 1, y_middle),(x_middle + 1, y_middle + 1),1)
			self.set_connected((x_middle, y_middle + 1),(x_middle + 1, y_middle + 1),1)

		# TODO: Prune some random walls to make the maze have more than 1 solution. I'll need to 
		# figure out a better way to generate the maze to reproduce the competition mazes. 

	def get_path(self, c1, c2):
		"""Returns a path between two cells
		
		Args:
		    c1 (tuple): start index [x,y]
		    c2 (tuple): end index [x,y]
		
		Returns:
		    List(int): the indices of the shortest path from start to end
		"""
		self.solve()
		c1_index = self.xy_to_index(c1)
		c2_index = self.xy_to_index(c2)

		if self.dists[c1_index, c2_index] > 0:
			path = []
			start = c1_index
			end = c2_index

			while start != end:
				if end < 0: 
					return -1 # No solution exists
				path.append(end)
				end = self.predecessors[start, end]

			path.append(start)
			return np.flipud(path)

		else: return []

	def xy_to_index(self, c):
		return c[0] + self.width * c[1]

	def index_to_xy(self, n):
		return np.array([n % self.width, np.floor(n / self.width)], dtype=int)

	def check_adjacent_index(self, c1_index, c2_index):
		c1 = self.index_to_xy(c1_index)
		c2 = self.index_to_xy(c2_index)
		return self.check_adjacent_xy(c1, c2)

	def check_adjacent_xy(self, c1, c2):
		return (abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])) == 1

	def __str__(self):
		cap = ''.join(['+---' for _ in range(self.width)]) + '+'
		output = cap + '\n'
		for y in range(self.height):
			lines = ['|' if self.get_connected([x,y], [x+1,y]) == 0
					     else ' ' for x in range(self.width - 1)]
			output += '|   ' + '   '.join(lines) + '   |\n'

			if y < self.height - 1:
				for x in range(self.width):
					output += '+---' if self.get_connected([x,y], [x,y+1]) == 0 else '+   '
				output += '+\n'

		output += cap
		return output


if __name__ == '__main__':
	m = Maze(3,2)
	# add some walls
	# m.set_connected([1,1],[0,1], 0)
	# m.set_connected([1,0],[1,1], 0)
	# see what it looks like
	print(m)

	# and get some of the resulting paths
	start, end = [0,0], [2,1]
	print('The path from {} to {} is {}'.format(start, end, m.get_path(start, end)))
	start, end = [2,1], [0,0]
	print('The path from {} to {} is {}'.format(start, end, m.get_path(start, end)))
	start, end = [0,1], [1,1]
	print('The path from {} to {} is {}'.format(start, end, m.get_path(start, end)))
