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

class Planner:
    """
    Intermediate planner that implements the Macaroni Planner (i.e., we either
    drive straight or drive in macaroni arcs to turn corners).
    """

    def __init__(self, maze):
        self.maze = maze
        self.curr_plan = None
        self.cells_in_plan = None
        self.calculate_traversal_times()
        
        self.first_quadrant_turns = set([PathType.TURN_BOTTOM_TO_LEFT,
                                         PathType.TURN_LEFT_TO_BOTTOM])
        self.second_quadrant_turns = set([PathType.TURN_RIGHT_TO_BOTTOM,
                                          PathType.TURN_BOTTOM_TO_RIGHT])
        self.third_quadrant_turns = set([PathType.TURN_TOP_TO_RIGHT,
                                         PathType.TURN_RIGHT_TO_TOP])
        self.fourth_quadrant_turns = set([PathType.TURN_LEFT_TO_TOP,
                                          PathType.TURN_TOP_TO_LEFT])
        
    def calculate_traversal_times(self):
        straight_velocity = 0.5  # meters/sec
        curve_velocity = 0.25  # meters/sec
        self.straight_time = p.maze_cell_size/straight_velocity
        self.curve_time = ((p.maze_cell_size * np.pi)/4.0)/curve_velocity
        self.angular_velocity = (np.pi/2)/self.curve_time
    
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
        self.generate_entry_times()
        
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
        x_extra = cell[1]*p.maze_cell_size - p.maze_wall_thickness/2.0
        y_extra = cell[0]*p.maze_cell_size - p.maze_wall_thickness/2.0
        # Treating the center of the dividing post at the bottom-left corner of
        # the current cell as the origin
        x_in_cell = x - x_extra
        y_in_cell = y - y_extra
        theta_in_cell = np.arctan(y_in_cell/x_in_cell)
        radius = p.maze_cell_size/2.0
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
        x_extra = (cell[1] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
        y_extra = cell[0]*p.maze_cell_size - p.maze_wall_thickness/2.0
        # Treating the center of the dividing post at the bottom-right corner of
        # the current cell as the origin
        x_in_cell = x - x_extra
        y_in_cell = y - y_extra
        theta_in_cell = np.arctan2(y_in_cell, x_in_cell)
        radius = p.maze_cell_size/2.0
        x_proj = radius*np.cos(theta_in_cell) + x_extra
        y_proj = radius*np.sin(theta_in_cell) + y_extra
        yaw_proj = theta_in_cell + np.pi/2.0
        if cell_path_type == PathType.TURN_BOTTOM_TO_RIGHT:
            # Going CW around the quarter circle
            yaw_proj -= np.pi
        return [x_proj, y_proj, yaw_proj]
        
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
        x_extra = (cell[1] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
        y_extra = (cell[0] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
        # Treating the center of the dividing post at the top-right corner of
        # the current cell as the origin
        x_in_cell = x - x_extra
        y_in_cell = y - y_extra
        theta_in_cell = np.arctan2(y_in_cell, x_in_cell)
        radius = p.maze_cell_size/2.0
        x_proj = radius*np.cos(theta_in_cell) + x_extra
        y_proj = radius*np.sin(theta_in_cell) + y_extra
        yaw_proj = theta_in_cell + np.pi/2.0
        if cell_path_type == PathType.TURN_RIGHT_TO_TOP:
            # Going CW around the quarter circle
            yaw_proj += np.pi
        return [x_proj, y_proj, yaw_proj]
        
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
        x_extra = cell[1]*p.maze_cell_size - p.maze_wall_thickness/2.0
        y_extra = (cell[0] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
        # Treating the center of the dividing post at the top-left corner of the
        # current cell as the origin
        x_in_cell = x - x_extra
        y_in_cell = y - y_extra
        theta_in_cell = np.arctan2(y_in_cell, x_in_cell)
        radius = p.maze_cell_size/2.0
        x_proj = radius*np.cos(theta_in_cell) + x_extra
        y_proj = radius*np.sin(theta_in_cell) + y_extra
        yaw_proj = theta_in_cell + np.pi/2.0
        if cell_path_type == PathType.TURN_TOP_TO_LEFT:
            # Going CW around the quarter circle
            yaw_proj -= np.pi
        return [x_proj, y_proj, yaw_proj]
        
    def _get_dt_in_cell(self, x, y, cell, cell_path_type):
        """
        Get the amount of time taken to reach the reference pose in the cell,
        relative to the entry time.
        """
        if cell_path_type == PathType.UP:
            # Vertical segment, so consider the y coordinate
            return abs(y - (cell[0]) * p.maze_cell_size)/self.straight_velocity
        elif cell_path_type == PathType.DOWN:
            # Vertical segment, so consider the y coordinate
            return abs(y - (cell[0] + 1) * p.maze_cell_size)/self.straight_velocity
        elif cell_path_type == PathType.LEFT:
            # Horizontal segment, so consider the x coordinate
            return abs(x - (cell[1] + 1) * p.maze_cell_size)/self.straight_velocity
        elif cell_path_type == PathType.RIGHT:
            # Horizontal segment, so consider the x coordinate
            return abs(x - (cell[1]) * p.maze_cell_size)/self.straight_velocity
        else:
        
            clockwise_turns = [PathType.TURN_BOTTOM_TO_RIGHT,
                               PathType.TURN_RIGHT_TO_TOP,
                               PathType.TURN_TOP_TO_LEFT,
                               PathType.TURN_LEFT_TO_BOTTOM]
            open_right_turns = [PathType.TURN_BOTTOM_TO_RIGHT,
                                PathType.TURN_RIGHT_TO_BOTTOM,
                                PathType.TURN_TOP_TO_RIGHT,
                                PathType.TURN_RIGHT_TO_TOP]
            open_up_turns = [PathType.TURN_TOP_TO_LEFT,
                             PathType.TURN_LEFT_TO_TOP,
                             PathType.TURN_TOP_TO_RIGHT,
                             PathType.TURN_RIGHT_TO_TOP]
            direction_of_rotation = cell_path_type in clockwise_turns
            direction_of_rotation = direction_of_rotation * 2 - 1
            center_post = cell
            center_post[0] += cell_path_type in open_right_turns
            center_post[1] += cell_path_type in open_up_turns
            center_post = center_post * p.maze_cell_size - p.maze_wall_thickness/2.0
            
            x_in_cell = x - center_post[0]
            y_in_cell = y - center_post[1]
            theta_in_cell = direction_of_rotation * p.arctan2(y_in_cell, x_in_cell)
            theta_in_cell = (theta_in_cell + np.pi) % (np.pi/2)
            return theta_in_cell/self.angular_velocity
        
    def get_t_on_path(self, pose):
        """
        Get the time at which this pose should occur on the path. Start time is
        at the start of the path.
        """
        curr_cell = self.maze.get_cell_from_global_xy(pose[0], pose[1])
        if curr_cell is None:
            print('ERROR: Actual pose not in the maze.')
            return None
        assert (curr_cell is not None)
        try:
            curr_cell_path_index = self.cells_in_plan.index(curr_cell)
            curr_cell_path_type = self.curr_plan[curr_cell_path_index]
        except ValueError:
            print('ERROR: Current cell not in the planned path.')
            return None
            
        curr_cell_entry_time = self.entry_times[curr_cell_path_index] +
                               self._get_dt_in_cell(pose[0], pose[1], curr_cell,
                                                    curr_cell_path_type)
                                                    
    def get_reference_pose_from_plan_by_time(self, t):
        """
        Given a time, which must be between the plan start time (t=0) and the
        plan end time, find the reference pose (x, y, yaw) corresponding to that
        time.
        
        This function is useful for getting the setpoint at some amount of time
        ahead of our current pose. Can call get_t_on_path() with the current
        pose, and then call this function with t=(get_t_on_path() + dt), where
        dt is the amount of time into the future that we want to set our
        setpoint.
        
        Return:
            a reference pose in [x, y, yaw] format
        """
        if t < 0:
            raise ValueError('Input time must be a non-negative number.')
        # Assuming constant velocity through a cell with straight path type
        # (Note that the last cell in a plan is always a straight path type)
        last_cell_duration = self.entry_times[-1] - self.entry_times[-2]
        if t >= self.entry_times[-1] - last_cell_duration/2.:
            # Return last reference pose in plan (have this pose be in the
            # center of the cell)
            print('WARNING: We do not have a plan far enough into the future '+
                  'for the input time. Returning a reference pose that is the '+
                  'center of the last cell of the plan.')
            cell = self.cells_in_plan[-1]
            cell_path_type = self.curr_plan[-1]
            x_proj = cell[1]*p.maze_cell_size + p.maze_inner_size/2.0
            y_proj = cell[0]*p.maze_cell_size + p.maze_inner_size/2.0
            if cell_path_type == PathType.RIGHT:
                # Point pose to the right
                yaw_proj = 0
            elif cell_path_type == PathType.LEFT:
                # Point pose to the left
                yaw_proj = np.pi
            elif cell_path_type == PathType.UP:
                # Point pose upward
                yaw_proj = np.pi/2.
            else:
                # Point pose downward
                yaw_proj = -np.pi/2.
            return [x_proj, y_proj, yaw_proj]
        
        for entry_time_index in range(len(self.entry_times)-1):
            if (t >= self.entry_times[entry_time_index] and
                    t < self.entry_times[entry_time_index + 1]):
                # We have found the cell in which the time falls. This condition
                # must occur at some point.
                cell_in_plan_index = entry_time_index
                cell = self.cells_in_plan[cell_in_plan_index]
                cell_path_type = self.curr_plan[cell_in_plan_index]
                break
                
        total_cell_duration = (self.entry_times[cell_in_plan_index + 1] -
                               self.entry_times[cell_in_plan_index])
        # Ratio of time spent in cell to total cell duration
        ratio = (t-self.entry_times[cell_in_plan_index]) / total_cell_duration
        
        if cell_path_type == PathType.UP:
            # Vertical segment, so determine the y coordinate by the time ratio
            x_proj = cell[1]*p.maze_cell_size + p.maze_inner_size/2.0
            y_proj = cell[0]*p.maze_cell_size + (ratio * p.maze_cell_size)
            yaw_proj = np.pi/2.
        elif cell_path_type == PathType.DOWN:
            # Vertical segment, so determine the y coordinate by the time ratio
            x_proj = cell[1]*p.maze_cell_size + p.maze_inner_size/2.0
            y_proj = (cell[0] + 1)*p.maze_cell_size - (ratio * p.maze_cell_size)
            yaw_proj = -np.pi/2.
        elif cell_path_type == PathType.LEFT:
            # Horizontal segment, so determine the x coordinate by the time ratio
            y_proj = cell[0]*p.maze_cell_size + p.maze_inner_size/2.0
            x_proj = (cell[1] + 1)*p.maze_cell_size - (ratio * p.maze_cell_size)
            yaw_proj = np.pi
        elif cell_path_type == PathType.RIGHT:
            # Horizontal segment, so determine the x coordinate by the time ratio
            y_proj = cell[0]*p.maze_cell_size + p.maze_inner_size/2.0
            x_proj = cell[1]*p.maze_cell_size + (ratio * p.maze_cell_size)
            yaw_proj = 0
        else:
            # Handle macaroni turns
            radius = p.maze_cell_size/2.0
            if cell_path_type in self.first_quadrant_turns:
                theta_in_cell = ratio*np.pi/2.
                # Treating the center of the dividing post at the bottom-left corner of
                # the current cell as the origin
                x_post = cell[1]*p.maze_cell_size - p.maze_wall_thickness/2.0
                y_post = cell[0]*p.maze_cell_size - p.maze_wall_thickness/2.0
            elif cell_path_type in self.second_quadrant_turns:
                theta_in_cell = ratio*np.pi/2. + np.pi/2.
                # Treating the center of the dividing post at the bottom-right corner of
                # the current cell as the origin
                x_post = (cell[1] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
                y_post = cell[0]*p.maze_cell_size - p.maze_wall_thickness/2.0
            elif cell_path_type in self.third_quadrant_turns:
                theta_in_cell = ratio*np.pi/2. + np.pi
                # Treating the center of the dividing post at the top-right corner of
                # the current cell as the origin
                x_post = (cell[1])*p.maze_cell_size - p.maze_wall_thickness/2.0
                y_post = (cell[0] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
            else:
                theta_in_cell = ratio*np.pi/2. + 3.*np.pi/2.
                # Treating the center of the dividing post at the top-left corner of
                # the current cell as the origin
                x_post = (cell[1] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
                y_post = (cell[0] + 1)*p.maze_cell_size - p.maze_wall_thickness/2.0
            x_proj = radius*np.cos(theta_in_cell) + x_post
            y_proj = radius*np.sin(theta_in_cell) + y_post
            yaw_proj = theta_in_cell + np.pi/2.0
        
        return [x_proj, y_proj, yaw_proj]
        
    def generate_entry_times(self):
        """
        Assuming constant velocities, generate the times at which we enter each
        cell. Also, the exit time of the last cell.
        """
        self.entry_times = []
        self.entry_times.append(0)
        for path_type in self.curr_plan:
            prev_entry_time = self.entry_times[-1]
            curr_entry_time = prev_entry_time + self.straight_time if path_type < 4 else self.curve_time
            self.entry_times.append(curr_entry_time)
            


if __name__ == '__main__':
    pass
