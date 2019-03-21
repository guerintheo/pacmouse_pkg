import numpy as np
import time
from pacmouse_pkg.src.utils.maze import Maze2

class Tremaux:
    def __init__(self, maze):
        self.markers = np.zeros(maze.width * maze.height)
        self.retracing = False

    def get_plan(self, cell, maze):
        self.markers[cell] += 1

        possible_cells = cell + np.array([-1, 1, -maze.width, maze.width])
        possible_cells = [c for c in possible_cells if maze.get_wall_between(cell, c) == 0]

        cell_counts = [self.markers[c] for c in possible_cells]
        return possible_cells[np.argmin(cell_counts)]


# helper function for simulated exploration of the maze. the robot is able
# to observe the walls around it.
def observe(cell, real_maze, estimated_maze):
    i_x = cell % real_maze.width
    i_y = np.floor(cell/real_maze.width).astype(int)
    estimated_maze.v_walls[i_x, i_y] = real_maze.v_walls[i_x, i_y]
    estimated_maze.v_walls[i_x+1, i_y] = real_maze.v_walls[i_x+1, i_y]
    estimated_maze.h_walls[i_x, i_y] = real_maze.h_walls[i_x, i_y]
    estimated_maze.h_walls[i_x, i_y+1] = real_maze.h_walls[i_x, i_y+1]

if __name__ == '__main__':
    # the ground truth maze which we're exploring
    w, h = 16, 16
    real_maze = Maze2(w,h)
    real_maze.generate_random_maze()
    print real_maze

    # the maze that we build as we explore
    estimated_maze = Maze2(w,h)
    print estimated_maze

    # position of the vehicle
    current_cell = 0
    target_cell = int(w * h/2)

    tremaux = Tremaux(real_maze)

    counter = 0
    while current_cell != target_cell:
        # we observe the cell we're currently in
        observe(current_cell, real_maze, estimated_maze)
        # step the tremaux planner
        current_cell = tremaux.get_plan(current_cell, estimated_maze)
        # print current_cell
        counter += 1

    print "We explored the maze in {} steps".format(counter)

