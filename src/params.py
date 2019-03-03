import numpy as np

# MAZE
maze_wall_thickness = 0.012 # m
maze_tolerance = 0.0005 # m
maze_inner_size = 0.168 # m (not including the thickness of the walls)
maze_cell_size = maze_inner_size + maze_wall_thickness

# ROBOT
lidar_transforms = np.array([[0.047535,	-0.038236,	-np.pi/2.0], # x, y, theta (body frame)
							 [0.051362,	-0.024628,	0.0],
							 [0.059890,	-0.009768,	-np.pi/4.0],
							 [0.059890,	0.009768,	np.pi/4.0],
							 [0.051362,	0.024628,	0.0],
							 [0.047535,	0.038236,	np.pi/2.0]])
robot_length = 0.107
robot_width = 0.0771

# Some geometric parameters for kinematics. These are just test values
wheel_radius = 0.01763
wheel_dist_x = 0.02
wheel_dist_y = 0.04
gear_ratio = 3*29.86

# these pin numbers are GPIO.BOARD
button_pins = [12,35,38,40]

# motor one is left (ml). motor two is right (mr)
ml_dir = 36
ml_pwm = 32
mr_dir = 26
mr_pwm = 33
