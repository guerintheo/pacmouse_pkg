import numpy as np
import pacmouse_pkg.src.params as p
from pacmouse_pkg.src.utils.math_utils import wrap
from pacmouse_pkg.src.utils.maze import Maze2


STRAIGHT = 0
LEFT = 1
RIGHT = 2

class Planner:

    def __init__(self, maze):
        self.maze = maze

    def make_plan(self, state, list_of_maze_indices):
        maze = self.maze
        

    def find_cell_type(self, list_of_next_cells):
        """
        Translates a list of cells to the type of the second cell. 
        list_of_next_cells: An np.array of 2 or 3 next cells, in [r,c] format. 

        Output: enum STRAIGHT, LEFT, RIGHT
        """
        if list_of_next_cells.shape[0] == 2:
            return STRAIGHT
        
        elif list_of_next_cells.shape[0] == 3:
            if (np.all(list_of_next_cells == list_of_next_cells[0,:], axis=0)[0] 
                or np.all(list_of_next_cells == list_of_next_cells[0,:], axis=0)[1]):
                return STRAIGHT
            
        return false


if __name__ == '__main__':
    m = Maze2(3,3)
    
    print(m)

    p = Planner(m)

    straight = p.find_cell_type(np.array([[0,1], [1,1], [2,1]]))
    print(straight)
