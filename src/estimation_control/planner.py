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
        self.curr_plan = None
        self.cells_in_plan = None

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
    
    def update_plan(self, cells):
        self.curr_plan = self.get_next_plan(cells)
        self.cells_in_plan = cells
        
    def get_reference_pose_from_plan(self, actual_pose):
        """
        Find the reference pose that the robot should be at according to the
        plan, based on the robot's actual pose. If in a straight path section,
        then the reference pose is simply the actual pose but centered in the
        maze corridor. For macaroni turns, we extend the pose radially toward
        the arc (a quarter-circle) that passes through the cell. Essentially, we
        are projecting the robot's actual pose onto the planned path.
        
        Args:
            actual_pose: a list consisting of [x, y, yaw]. x and y are
            global-frame coordinates given in meters. [0,0] is at the bottom
            left corner of the maze (the bottom left corner of cell 0), and
            positive x increases as you move to the right along a row, while
            positive y increases as you move up along a column. Yaw angle is
            given in radians as the rotation of the body frame relative to the
            global frame. The robot's body-frame x-axis points forward, and its
            body-frame y-axis points to the left; as such, yaw angle is 0 when
            the robot is pointing to the right (i.e., along the global-frame
            x-axis) and is pi/2 when the robot is pointing up along a column
            (i.e., along the global-frame y-axis).
        """
        curr_cell = self.maze.get_cell_from_global_xy(actual_pose[0], actual_pose[1])
        if curr_cell is None:
            print('ERROR: Actual pose not in the maze.')
            return None
        assert (curr_cell is not None)
        try:
            curr_cell_path_type = self.curr_plan[self.cells_in_plan.index(curr_cell)]
        except ValueError:
            print('ERROR: Current cell not in the planned path.')
            return None
        
        # Project onto straight segments if applicable
        if curr_cell_path_type == PathType.UP or curr_cell_path_type == PathType.DOWN:
            return self._project_pose_on_vertical_segment(actual_pose[0],
                                                          actual_pose[1],
                                                          curr_cell,
                                                          curr_cell_path_type)
        if curr_cell_path_type == PathType.LEFT or curr_cell_path_type == PathType.RIGHT:
            return self._project_pose_on_horizontal_segment(actual_pose[0],
                                                            actual_pose[1],
                                                            curr_cell,
                                                            curr_cell_path_type)
            
        # Project onto macaroni segments if applicable
        if (curr_cell_path_type == PathType.TURN_BOTTOM_TO_LEFT
                or curr_cell_path_type == PathType.TURN_LEFT_TO_BOTTOM):
            # Macaroni is first quadrant of a circle
            return self._project_pose_on_first_quadrant_macaroni(actual_pose[0],
                                                                 actual_pose[1],
                                                                 curr_cell,
                                                                 curr_cell_path_type)
        if (curr_cell_path_type == PathType.TURN_BOTTOM_TO_RIGHT
                or curr_cell_path_type == PathType.TURN_RIGHT_TO_BOTTOM):
            # Macaroni is second quadrant of a circle
            return self._project_pose_on_second_quadrant_macaroni(actual_pose[0],
                                                                  actual_pose[1],
                                                                  curr_cell,
                                                                  curr_cell_path_type)
        if (curr_cell_path_type == PathType.TURN_TOP_TO_RIGHT
                or curr_cell_path_type == PathType.TURN_RIGHT_TO_TOP):
            # Macaroni is third quadrant of a circle
            return self._project_pose_on_third_quadrant_macaroni(actual_pose[0],
                                                                 actual_pose[1],
                                                                 curr_cell,
                                                                 curr_cell_path_type)
        else:
            # Macaroni is fourth quadrant of a circle
            return self._project_pose_on_fourth_quadrant_macaroni(actual_pose[0],
                                                                  actual_pose[1],
                                                                  curr_cell,
                                                                  curr_cell_path_type)
            
    def _project_pose_on_vertical_segment(self, x, y, cell, cell_path_type):
        """
        Args:
            x: the global-frame x coordinate of the pose we are projecting
            y: the global-frame y coordinate of the pose we are projecting
            cell: the current cell, in [r, c] format, of the position x,y
            cell_path_type: the PathType of cell
            
        Returns:
            The projected pose: a list consisting of [x_proj, y_proj, yaw_proj]
        """
        x_proj = cell[1]*p.maze_cell_size + p.maze_inner_size/2.0
        y_proj = y  # y coordinate does not change
        if cell_path_type == PathType.UP:
            # Point pose upward
            yaw_proj = np.pi/2
        else:
            # Point pose downward
            yaw_proj = -np.pi/2
        return [x_proj, y_proj, yaw_proj]
        
    def _project_pose_on_horizontal_segment(self, x, y, cell, cell_path_type):
        """
        Args:
            x: the global-frame x coordinate of the pose we are projecting
            y: the global-frame y coordinate of the pose we are projecting
            cell: the current cell, in [r, c] format, of the position x,y
            cell_path_type: the PathType of cell
            
        Returns:
            The projected pose: a list consisting of [x_proj, y_proj, yaw_proj]
        """
        x_proj = x  # x coordinate does not change
        y_proj = cell[0]*p.maze_cell_size + p.maze_inner_size/2.0
        if cell_path_type == PathType.RIGHT:
            # Point pose to the right
            yaw_proj = 0
        else:
            # Point pose to the left
            yaw_proj = np.pi
        return [x_proj, y_proj, yaw_proj]
    
    def _project_pose_on_first_quadrant_macaroni(self, x, y, cell, cell_path_type):
        """
        Args:
            x: the global-frame x coordinate of the pose we are projecting
            y: the global-frame y coordinate of the pose we are projecting
            cell: the current cell, in [r, c] format, of the position x,y
            cell_path_type: the PathType of cell
            
        Returns:
            The projected pose: a list consisting of [x_proj, y_proj, yaw_proj]
        """
        x_extra = cell[1]*p.maze_cell_size
        y_extra = cell[0]*p.maze_cell_size
        # Treating the bottom-left corner of the current cell as the origin
        x_in_cell = x - x_extra
        y_in_cell = y - y_extra
        theta_in_cell = np.arctan(y_in_cell/x_in_cell)
        radius = p.maze_inner_size
        x_proj = radius*np.cos(theta_in_cell) + x_extra
        y_proj = radius*np.sin(theta_in_cell) + y_extra
        yaw_proj = theta_in_cell + np.pi/2.0
        if cell_path_type == PathType.TURN_LEFT_TO_BOTTOM:
            # Going CW around the quarter circle
            yaw_proj += np.pi
        return [x_proj, y_proj, yaw_proj]
        
    def _project_pose_on_second_quadrant_macaroni(self, x, y, cell, cell_path_type):
        """
        Args:
            x: the global-frame x coordinate of the pose we are projecting
            y: the global-frame y coordinate of the pose we are projecting
            cell: the current cell, in [r, c] format, of the position x,y
            cell_path_type: the PathType of cell
            
        Returns:
            The projected pose: a list consisting of [x_proj, y_proj, yaw_proj]
        """
        # TODO
        pass
        
    def _project_pose_on_third_quadrant_macaroni(self, x, y, cell, cell_path_type):
        """
        Args:
            x: the global-frame x coordinate of the pose we are projecting
            y: the global-frame y coordinate of the pose we are projecting
            cell: the current cell, in [r, c] format, of the position x,y
            cell_path_type: the PathType of cell
            
        Returns:
            The projected pose: a list consisting of [x_proj, y_proj, yaw_proj]
        """
        # TODO
        pass
        
    def _project_pose_on_fourth_quadrant_macaroni(self, x, y, cell, cell_path_type):
        """
        Args:
            x: the global-frame x coordinate of the pose we are projecting
            y: the global-frame y coordinate of the pose we are projecting
            cell: the current cell, in [r, c] format, of the position x,y
            cell_path_type: the PathType of cell
            
        Returns:
            The projected pose: a list consisting of [x_proj, y_proj, yaw_proj]
        """
        # TODO
        pass


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

