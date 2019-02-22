import numpy as np
import params as p
from util import wrap

def step(x, sp):
	"""Generates commands to drive the robot to a desired pose
	
	Args:
	    x (1d numpy array): a 6-vector of the state of the bot
	    sp (1d numpy array): a 6-vector of the target state (set point)
	
	Returns:
	    1d numpy array: a 2-vector [drive, steer]
	"""
	steering_coeff = 100.
	drive_coeff    = 1000.

	e_x, e_y = sp[:2] - x[:2] 					# the position difference
	e_t = wrap(np.arctan2(e_y, e_x) - x[2]) 	# the angular error to point at the desired position

	d = np.sqrt(e_x**2 + e_y**2)				# the distance to the desired position
	facing_target = (np.pi - np.abs(e_t))/np.pi # 1 when facing the target. 0 when facing away from target

	steering_command = e_t * steering_coeff				# turn to face the target
	drive_command =  d * facing_target * drive_coeff	# drive forward faster when facing the target

	return np.array([drive_command, steering_command])


def get_sp(x, maze, target_cell):
	"""Find a path from the current position in the maze to the target cell.
	Returns a set point for the robot to drive to.
	
	Args:
	    x (1d numpy array): a 6-vector of the state of the bot
	    maze (Maze): The maze the robot is working in (instance of Maze class)
	    target_cell (1d numpy array): coordinates of the goal cell in the maze
	
	Returns:
	    1d numpy array: The coordinates that the robot should drive to
	"""
	cell_coord = np.floor(x[:2] / p.maze_inner_size).astype(int)
	path = maze.get_path(cell_coord, target_cell)
	target_theta = 0

	if len(path) < 2:
		print 'We made it!'
		return x
	elif len(path) > 2:
		dx, dy = maze.index_to_xy(path[2]) - maze.index_to_xy(path[1])
		target_theta = np.arctan2(dy, dx)

	tx, ty = (maze.index_to_xy(path[1]) + 0.5) * p.maze_inner_size
	
	return np.array([tx, ty, target_theta])

def mix(u):
	"""Given a command [drive, steer], return wheel speeds
	
	Args:
	    u (1d numpy array): a 2-vector [drive, steer]
	
	Returns:
	    1d numpy array: a 2-vector [left, right] wheel speeds
	"""
	a = np.array([[1,-1],
				  [1, 1]])
	return np.dot(a, u)
