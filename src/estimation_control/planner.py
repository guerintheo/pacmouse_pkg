import numpy as np
from enum import Enum

import pacmouse_pkg.src.params as p
from pacmouse_pkg.src.utils.math_utils import wrap
from pacmouse_pkg.src.utils.maze import Maze2


class PathType(Enum):
    # Straight paths:
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3
    
    # Macaroni arc paths:
    TURN_BOTTOM_TO_RIGHT = 4
    TURN_RIGHT_TO_BOTTOM = 5
    TURN_RIGHT_TO_TOP = 6
    TURN_TOP_TO_RIGHT = 7
    TURN_TOP_TO_LEFT = 8
    TURN_LEFT_TO_TOP = 9
    TURN_LEFT_TO_BOTTOM = 10
    TURN_BOTTOM_TO_LEFT = 11
    
    # # Only plan from the incoming edge to the middle of the cell (i.e., don't commit
    # # to planning a straight path all of the way through the cell). This designation
    # # is useful for planning the path of the last cell in a plan, as we don't know
    # # where to go next.
    # STRAIGHT_TO_MIDDLE = 3 

class Planner:
    """
    Intermediate planner that implements the Macaroni Planner (i.e., we either
    drive straight or drive in macaroni arcs to turn corners).
    """

    def __init__(self, maze):
        self.maze = maze

    def make_plan(self, state, list_of_maze_indices):
        maze = self.maze
        
    # def get_next_poses(self, curr_pose, num_poses):
    #     """
    #     Return a list of set points / reference poses for the robot to try to
    #     track.
    #     """
    
    def get_next_plan(self, cells):
        """
        Given the current cell of the robot and future cells to explore, return
        a list of path types (see the enumerations in the PathType class),
        each one corresponding to a cell.

        Args:
            cells: a list of adjacaent cells as two-item lists in [r, c] format,
                   where r is the row number and c is the column number. The
                   first cell in the list is the current cell of the robot. The
                   entire list consists of the cells over which to form a plan.
                   The list must be at least two cells long.
                   
        Returns:
            A list of PathType enums that describe the path to traverse the
            cells in the input list
            
        ------------------------------------------------------------------------
        
        Example:
            Input:
                cells: [[0, 0], [1, 0], [2, 0], [2, 1]]
            Output:
                [PathType.UP,
                 PathType.UP,
                 PathType.TURN_BOTTOM_TO_RIGHT,
                 PathType.RIGHT]
        """
        assert len(cells) >= 2
        path = []
        # Determine first cell path type
        cell1 = cells[0]
        cell2 = cells[1]
        path.append(self.determine_direction_between_cells(cell1, cell2))
        
        for cell_index in range(len(cells)-2):
            cell_trio = [cells[cell_index], cells[cell_index+1], cells[cell_index+2]]
            path.append(self.find_middle_cell_type(cell_trio))
        
        # Determine the last cell type
        path.append(self.determine_direction_between_cells(cells[-2], cells[-1]))
        return path
        
    def find_middle_cell_type(self, cells):
        """
        Given a list of three adjacent cells, each in [r, c] format, return the
        PathType of the middle cell.
        """
        assert len(cells) == 3
        
        c1 = cells[0]
        c2 = cells[1]
        c3 = cells[2]
        
        # Check for straight connection
        if (c1[0] == c2[0] == c3[0]) or (c1[1] == c2[1] == c3[1]):
            # The three cells are in the same row or same column
            return self.determine_direction_between_cells(c2, c3)
        
        # Check for a turn
        if c1[0] < c3[0] and c1[1] < c3[1]:
            if c2[0] == c3[0]:
                return PathType.TURN_BOTTOM_TO_RIGHT
            else:
                return PathType.TURN_LEFT_TO_TOP
        if c1[0] > c3[0] and c1[1] > c3[1]:
            if c2[0] == c3[0]:
                return PathType.TURN_TOP_TO_LEFT
            else:
                return PathType.TURN_RIGHT_TO_BOTTOM
        if c1[0] < c3[0] and c1[1] > c3[1]:
            if c2[0] == c3[0]:
                return PathType.TURN_BOTTOM_TO_LEFT
            else:
                return PathType.TURN_RIGHT_TO_TOP
        if c1[0] > c3[0] and c1[1] < c3[1]:
            if c2[0] == c3[0]:
                return PathType.TURN_TOP_TO_RIGHT
            else:
                return PathType.TURN_LEFT_TO_BOTTOM
    
    def determine_direction_between_cells(self, cell1, cell2):
        assert cell1 != cell2
        # cell1 and cell2 must be adjacent cells
        if cell1[0] < cell2[0]:
            return PathType.UP
        elif cell1[0] > cell2[0]:
            return PathType.DOWN
        elif cell1[1] < cell2[1]:
            return PathType.RIGHT
        else:
            return PathType.LEFT
        


if __name__ == '__main__':
    m = Maze2(4,4)
    #m.generate_random_maze()
    m.set_wall_between(0, 1, 1)
    m.set_wall_between(4, 5, 1)
    m.set_wall_between(8, 12, 1)
    m.set_wall_between(9, 13, 1)
    m.set_wall_between(5, 6, 1)
    m.set_wall_between(6, 10, 1)
    m.set_wall_between(11, 15, 1)
    print(m)

    p = Planner(m)
    plan = p.get_next_plan([[0, 0], [1, 0], [2, 0], [2, 1]])
    print plan

