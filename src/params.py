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

# Some geometric parameters for kinematics
wheel_radius = 0.01763
wheel_dist_x = 0.02
wheel_dist_y = 0.04

motor_gear_ratio = 29.86
wheel_gear_ratio = 3
gear_ratio = wheel_gear_ratio * motor_gear_ratio

motor_pwm_freq = 100

# these pin numbers are GPIO.BOARD
button_pins = [12,35,38,40]

# motor one is left (ml). motor two is right (mr)
ml_dir = 36
ml_pwm = 32
mr_dir = 26
mr_pwm = 33
motor_mode_pin = 22

motor_pins = [ml_dir, ml_pwm, mr_dir, mr_pwm, motor_mode_pin]

enc_l_a = 16
enc_l_b = 18
enc_r_a = 15
enc_r_b = 13

encoder_pins = [enc_l_a, enc_l_b, enc_r_a, enc_r_b]