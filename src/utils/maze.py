import sys
import numpy as np
from scipy.sparse.csgraph import dijkstra
import pacmouse_pkg.src.params as p

class Maze:

    """Stores a representation of the maze.

    The maze is indexed by [x,y] (increasing to the right, increasing down)
    coordinates with the 0,0 cell in the top left corner.

    The maze is stored as an adjacency matrix. A value of 1 in the matrix
    indicates that the cells are connected. A value of zero indicates that
    the cells are disconnected.

    Attributes:
        height (int): The width of the Maze
        width (int): The height of the Maze
        filename (String): filename
        adj_matrix (2d-nparray): adjacency matrix to start with
    """

    def __init__(self, width=16, height=16, filename=None, adj_matrix=None):
        if filename is not None:
            self.load(filename)
        else:
            self.width = width
            self.height = height

            if adj_matrix is not None:
                self.adj_matrix = adj_matrix
            else:
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
        return int(c[0] + self.width * c[1])

    def index_to_xy(self, n):
        # i.e., outputs col,row format
        return np.array([n % self.width, np.floor(n / self.width)], dtype=int)

    def check_adjacent_index(self, c1_index, c2_index):
        c1 = self.index_to_xy(c1_index)
        c2 = self.index_to_xy(c2_index)
        return self.check_adjacent_xy(c1, c2)

    def check_adjacent_xy(self, c1, c2):
        return (abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])) == 1


    def build_segment_list(self):
        # each segment is [x_start, y_start, x_end, y_end, adj]
        self.segment_list = []
        c = p.maze_cell_size
        w = p.maze_wall_thickness/2.
        for x in range(self.width):
            for y in range(self.height):
                if x < self.width - 1:
                    adj = self.get_connected([x,y], [x+1,y])
                    self.segment_list.append([(x+1)*c-w, y*c-w, (x+1)*c-w, (y+1)*c+w, adj])
                    self.segment_list.append([(x+1)*c+w, y*c-w, (x+1)*c+w, (y+1)*c+w, adj])
                if y < self.height - 1:
                    adj = self.get_connected([x,y], [x,y+1])
                    self.segment_list.append([x*c-w, (y+1)*c-w, (x+1)*c+w, (y+1)*c-w, adj])
                    self.segment_list.append([x*c-w, (y+1)*c+w, (x+1)*c+w, (y+1)*c+w, adj])

        # inner walls
        self.segment_list.append([w, w, self.width*c-w, w, 0])
        self.segment_list.append([w, w, w, self.height*c-w, 0])
        self.segment_list.append([self.width*c-w,w, self.width*c-w, self.height*c-w, 0])
        self.segment_list.append([w, self.height*c-w, self.width*c-w, self.height*c-w, 0])

        # outer walls
        self.segment_list.append([-w, -w, self.width*c+w, -w, 0])
        self.segment_list.append([-w, -w, -w, self.height*c+w, 0])
        self.segment_list.append([self.width*c+w,-w, self.width*c+w, self.height*c+w, 0])
        self.segment_list.append([-w, self.height*c+w, self.width*c+w, self.height*c+w, 0])

    def plot(self, plt, color='k'):
        for seg in self.segment_list:
            plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k', color=color, alpha=1-seg[4])

    def get_v_wall(self, x, y):
        """access the vertical walls of the maze as if they were in a width+1
        by height array (including the outer walls)

        Args:
            x (int): x index of vertical wall
            y (int): y index of vertical wall

        Returns:
            float:  (0,1) representing the connectedness (absense of a wall)
        """
        assert 0 <= x <= self.width
        assert 0 <= y < self.height
        if x == 0 or x == self.width:
            return 0
        else:
            return self.get_connected([x-1,y], [x,y])

    def get_h_wall(self, x, y):
        """access the horizontal walls of the maze as if they were in a width
        by height+1 array (including the outer walls)

        Args:
            x (int): x index of horizontal wall
            y (int): y index of horizontal wall

        Returns:
            float:  (0,1) representing the connectedness (absense of a wall)
        """
        assert 0 <= x < self.width
        assert 0 <= y <= self.height
        if y == 0 or y == self.height:
            return 0
        else:
            return self.get_connected([x,y-1], [x,y])

    def build_wall_matrices(self):
        h_wall_indices = np.arange(self.width * (self.height+1))
        v_wall_indices = np.arange((self.width+1) * self.height)

        self.h_walls = np.zeros([self.width, self.height+1])
        for i in range(self.width):
            for j in range(self.height + 1):
                self.h_walls[i,j] = self.get_h_wall(i,j)

        self.v_walls = np.zeros([self.width+1, self.height])
        for i in range(self.width + 1):
            for j in range(self.height):
                self.v_walls[i,j] = self.get_v_wall(i,j)

        # add the perimeter walls
        self.v_walls[0, :] = 1
        self.v_walls[-1, :] = 1
        self.h_walls[:, 0] = 1
        self.h_walls[:, -1] = 1
        
    def get_cell_from_global_xy(self, x, y):
        """
        Return the cell corresponding to the global x,y coordinates. Return None
        if the coordinates fall outside of the maze. If the coordinates land on
        a wall, choose the cell whose center is closest. Return cell in [r, c]
        format.
        """
        # Shift origin to middle of the intersection of the walls in the bottom
        # left corner of the maze
        x = x + p.maze_wall_thickness/2.0
        y = y + p.maze_wall_thickness/2.0
        # int() truncates the value
        row = int(y/p.maze_cell_size)
        col = int(x/p.maze_cell_size)
        if (row > self.height - 1) or (col > self.width - 1):
            return None
        return [row, col]

    def __str__(self):
        cap = ''.join(['+---' for _ in range(self.width)]) + '+'
        output = cap + '\n'
        for y in range(self.height-1,-1,-1):
            lines = ['|' if self.get_connected([x,y], [x+1,y]) == 0
                         else ' ' for x in range(self.width - 1)]
            output += '|   ' + '   '.join(lines) + '   |\n'

            if y > 0:
                for x in range(self.width):
                    output += '+---' if self.get_connected([x,y], [x,y-1]) == 0 else '+   '
                output += '+\n'

        output += cap
        return output


class Maze2:
    def __init__(self, width=16, height=16):
        self.width = width
        self.height = height
        self.create_empty_maze()
        self.add_perimeter()

    def create_empty_maze(self):
        self.h_walls = np.zeros([self.width, self.height+1])
        self.v_walls = np.zeros([self.width+1, self.height])

    def add_perimeter(self):
        self.v_walls[[0,-1],:] = 1
        self.h_walls[:,[0,-1]] = 1

    def build_segment_list(self):
        # each segment is [x_start, y_start, x_end, y_end, wall]
        self.segment_list = []
        c = p.maze_cell_size
        w = p.maze_wall_thickness/2.
        for x in range(self.width+1):
            for y in range(self.height+1):
                if y < self.height:
                    self.segment_list.append([(x)*c-w, y*c-w, (x)*c-w, (y+1)*c+w, self.v_walls[x,y]])
                    self.segment_list.append([(x)*c+w, y*c-w, (x)*c+w, (y+1)*c+w, self.v_walls[x,y]])
                if x < self.width:
                    self.segment_list.append([x*c-w, (y)*c-w, (x+1)*c+w, (y)*c-w, self.h_walls[x,y]])
                    self.segment_list.append([x*c-w, (y)*c+w, (x+1)*c+w, (y)*c+w, self.h_walls[x,y]])

    def plot(self, plt, color='k', confidence=True):
        for seg in self.segment_list:
            plt.plot((seg[0], seg[2]), (seg[1], seg[3]), 'k',
                     color=color, alpha=(seg[4] if confidence else 1))

    def build_adjacency_matrix(self, threshold=None):
        self.adj_matrix = np.zeros([self.width*self.height, self.width*self.height])
        w = self.width
        h = self.height
        for x in range(w):
            for y in range(h):
                # check cell to the right
                if x < w - 1:
                    i = y*w + x
                    j = i + 1
                    connected = 1. - self.v_walls[x+1, y]
                    if threshold is not None:
                        connected = connected > threshold
                    self.adj_matrix[i, j] = self.adj_matrix[j, i] = connected

                # check cell above
                if y < h - 1:
                    i = y*w + x
                    j = i + w
                    connected = 1. - self.h_walls[x, y+1]
                    if threshold is not None:
                        connected = connected > threshold
                    self.adj_matrix[i, j] = self.adj_matrix[j, i] = connected

    def solve(self):
        self.dists, self.predecessors = dijkstra(self.adj_matrix, directed=False,
                                        unweighted=True, return_predecessors=True)

    def get_path(self, c1, c2):
        c1_index = c1[1] * self.width + c1[0]
        c2_index = c2[1] * self.width + c2[0]

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

    def route(self, c1, c2, threshold=None):
        self.build_adjacency_matrix(threshold=threshold)
        self.solve()
        return self.get_path(c1, c2)

    def __str__(self):
        output = ""
        for y in range(self.height, -1, -1):
            # build v_wall
            if y < self.height:
                walls = ['|' if self.v_walls[x,y] else ' ' for x in range(self.width+1)]
                output += '   '.join(walls) + '\n'

            # build h_wall
            walls = ['---' if self.h_walls[x,y] else '   ' for x in range(self.width)]
            output += '+' + '+'.join(walls) + '+\n'
        return output[:-1]

    def load(self, filename):
        with open(filename, 'r') as f:
            self.parse(f.read())

    def save(self, filename):
        with open(filename, 'w') as f:
            f.write(self.__str__())

    def parse(self, s):
        lines = s.split('\n')
        assert len(lines)%2 == 1 # the number of lines should be odd
        assert len(lines[0])%4 ==1 # each cell is 3 characters wide

        self.height = h = (len(lines)-1)/2
        self.width = w = (len(lines[0])-1)/4
        self.v_walls = np.zeros([w+1, h])
        self.h_walls = np.zeros([w, h+1])

        for i, l in enumerate(lines):
            if i%2 == 0: # we are dealing with a horizontal wall row
                self.h_walls[:, h-i/2] = [l[j] == '-' for j in np.arange(w)*4+2]
            else: # we are handling a vertical wall row
                self.v_walls[:, h - (i+1)/2] =  [l[j] == '|' for j in np.arange(w+1)*4]

    def get_wall_between(self, i1, i2):
        w = self.width
        c1 = [i1%w, np.floor(i1/w).astype(int)]
        c2 = [i2%w, np.floor(i2/w).astype(int)]
        if c1[0] == c2[0]: return self.h_walls[c1[0], max(c1[1], c2[1])]
        elif c1[1] == c2[1]: return self.v_walls[max(c1[0], c2[0]), c1[1]]
        else: 
            return -1

    def is_adjacent(self, i1, i2):
        w = self.width
        c1 = np.array([i1%w, np.floor(i1/w).astype(int)])
        c2 = np.array([i2%w, np.floor(i2/w).astype(int)])
        return np.any(c1 == c2)

    def set_wall_between(self, i1, i2, is_wall):
        w = self.width
        c1 = [i1%w, np.floor(i1/w).astype(int)]
        c2 = [i2%w, np.floor(i2/w).astype(int)]
        if c1[0] == c2[0]: self.h_walls[c1[0], max(c1[1], c2[1])] = is_wall
        elif c1[1] == c2[1]: self.v_walls[max(c1[0], c2[0]), c1[1]] = is_wall
        else: print 'ERROR: {} and {} are not adjacent'.format(c1, c2)

    def generate_random_maze(self, prune_walls=0.1):
        self.v_walls[:,:] = 1
        self.h_walls[:,:] = 1
        visited = set()

        def generate_random_maze_sub(self, cell):
            visited.add(cell) 
            next_cells = cell + np.array([-1, 1, -self.width, self.width])
            np.random.shuffle(next_cells)
            for next_cell in next_cells:
                if 0 <= next_cell < (self.width * self.height) and\
                        self.is_adjacent(cell, next_cell) and\
                        next_cell not in visited:
                    self.set_wall_between(cell, next_cell, 0)
                    generate_random_maze_sub(self, next_cell)

        generate_random_maze_sub(self, 0)

        self.h_walls *= (np.random.rand(*self.h_walls.shape) > prune_walls)
        self.v_walls *= (np.random.rand(*self.v_walls.shape) > prune_walls)
        self.add_perimeter()

    def get_cell_from_global_xy(self, x, y):
        """
        Return the cell corresponding to the global x,y coordinates. Return None
        if the coordinates fall outside of the maze. If the coordinates land on
        a wall, choose the cell whose center is closest. Return cell in [r, c]
        format.
        """
        # Shift origin to middle of the intersection of the walls in the bottom
        # left corner of the maze
        x = x + p.maze_wall_thickness/2.0
        y = y + p.maze_wall_thickness/2.0
        # int() truncates the value
        row = int(y/p.maze_cell_size)
        col = int(x/p.maze_cell_size)
        if (row > self.height - 1) or (col > self.width - 1):
            return None
        return [row, col]
        
        


if __name__ == '__main__':
    m = Maze2(4,3)
    m.generate_random_maze()
    print m
    m.save('test.maze')
    print 'saved'
    n = Maze2(1,1)
    n.load('test.maze')
    print 'loaded'
    print n

    # if len(sys.argv) > 1 and sys.argv[1] == '2':
    #     m = Maze2(16,16)
    #     # m.h_walls[0,1] = 1
    #     # m.h_walls[1,2] = 1
    #     # print m.h_walls
    #     # print m.v_walls
    #     # m.build_adjacency_matrix()
    #     # print m.adj_matrix
    #     # print m
    #     m.generate_random_maze()
    #     print m

    # else:
    #     m = Maze(3,2)
    #     # add some walls
    #     # m.set_connected([1,1],[0,1], 0)
    #     # m.set_connected([1,0],[1,1], 0)
    #     # see what it looks like
    #     print(m)

    #     # and get some of the resulting paths
    #     start, end = [0,0], [2,1]
    #     print('The path from {} to {} is {}'.format(start, end, m.get_path(start, end)))
    #     start, end = [2,1], [0,0]
    #     print('The path from {} to {} is {}'.format(start, end, m.get_path(start, end)))
    #     start, end = [0,1], [1,1]
    #     print('The path from {} to {} is {}'.format(start, end, m.get_path(start, end)))
