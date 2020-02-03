import numpy as np

# MAZE
maze_wall_thickness = 0.012  # m
maze_tolerance = 0.0005  # m

# Not including the thickness of the walls. This is the interior dimension of
# a cell
maze_inner_size = 0.168  # m
maze_cell_size = maze_inner_size + maze_wall_thickness
wall_height = .050
wall_uri = 'model://maze_wall'

# ROBOT
chassis_mass_kg = 0.1  # TODO: Get actual value
chassis_width = 0.0335  # meters
chassis_length = 0.073572  # meters
# chassis_height = 0.037260  # meters
chassis_height = 0.025  # meters  # TODO: Get actual value
chassis_vertical_offset = 0.005393  # meters
chassis_forward_offset = 0.008926  # meters

lidar_pins = [4, 17, 10, 11, 9, 5]  # These pin numbers are BCM

# Each row is [x, y, theta (body frame)]
lidar_transforms = np.array([[0.047535, -0.038236, -np.pi / 2.0],
                             [0.051362, -0.024628, 0.0],
                             [0.059890, -0.009768, -np.pi / 4.0],
                             [0.059890, 0.009768, np.pi / 4.0],
                             # [0.051362,	0.024628,	0.0],
                             [0.047535, 0.038236, np.pi / 2.0]])
num_lidars = lidar_transforms.shape[0]
robot_length = 0.107
robot_width = 0.0771

# Some geometric parameters for kinematics
wheel_radius = 0.01763  # meters
wheel_rim_inner_radius = wheel_radius - 0.002  # TODO: Get actual value
wheel_dist_x = 0.02  # meters
wheel_dist_y = 0.04  # meters

wheel_width = 0.009144  # meters
wheel_front_offset = 0.018382  # meters
wheel_rear_offset = -0.018382  # meters
wheel_lateral_offset = 0.033498  # meters
wheel_vertical_offset = 0.0  # meters  # TODO: Get actual value

wheel_mass_kg = 0.03  # TODO: Get actual value
wheel_surface_friction = 0.8  # TODO: Determine reasonable value for simulation


def compute_chassis_inertia():
    # Compute chassis inertia matrix modeled as a rectangle
    # Refer to https://www.brown.edu/Departments/Engineering/Courses/En4/Notes/
    #          Rigid_Bodies/Rigid_Bodies.pdf
    mass_prefactor = chassis_mass_kg/12.0
    ixx = mass_prefactor * (chassis_width**2.0 + chassis_height**2.0)
    ixy = 0.0
    ixz = 0.0
    iyy = mass_prefactor * (chassis_length**2.0 + chassis_height**2.0)
    iyz = 0.0
    izz = mass_prefactor * (chassis_length**2.0 + chassis_width**2.0)
    return [ixx, ixy, ixz, iyy, iyz, izz]


chassis_ixx, chassis_ixy, chassis_ixz, chassis_iyy, chassis_iyz, chassis_izz \
    = compute_chassis_inertia()


def compute_wheel_inertia():
    # Compute wheel inertia matrix modeled as a hollow cylinder
    # Refer to https://www.brown.edu/Departments/Engineering/Courses/En4/Notes/
    #          Rigid_Bodies/Rigid_Bodies.pdf
    mass_prefactor = chassis_mass_kg / 12.0
    ixx = mass_prefactor * (wheel_width**2.0 + 3*(wheel_rim_inner_radius**2.0
                                                  + wheel_radius**2.0))
    ixy = 0.0
    ixz = 0.0
    iyy = ixx
    iyz = 0.0
    izz = mass_prefactor * (6*(wheel_rim_inner_radius**2.0 + wheel_radius**2.0))
    return [ixx, ixy, ixz, iyy, iyz, izz]


wheel_ixx, wheel_ixy, wheel_ixz, wheel_iyy, wheel_iyz, wheel_izz \
    = compute_wheel_inertia()

motor_gear_ratio = 29.86
wheel_gear_ratio = 3
gear_ratio = wheel_gear_ratio * motor_gear_ratio

motor_pwm_freq = 100

# These pin numbers are GPIO.BOARD
# button_pins = [12,35,38,40]
# These pin numbers are BCM
button_1 = 18
button_2 = 19
button_3 = 20
button_4 = 21
button_pins = [button_1, button_2, button_3, button_4]

######################### MOTOR PINS ###########################################
# NOTE: These pin numbers are BCM (Broadcom), not GPIO.BOARD / physical pin
# numbers. pigpio uses these BCM pin numbers. Refer to https://pinout.xyz/ to
# see the pinout laid out.

# motor one is left (ml). motor two is right (mr)
ml_dir = 7  # corresponds to physical pin 26
ml_pwm = 13  # corresponds to physical pin 33
mr_dir = 16  # bcm pin 16 corresponds to physical pin 36
mr_pwm = 12  # bcm pin 12 corresponds to physical pin 32
motor_mode_pin = 25  # corresponds to physical pin 22

motor_pins = [ml_dir, ml_pwm, mr_dir, mr_pwm, motor_mode_pin]

######################### ENCODER PINS ###########################################

# These pin numbers are BCM
enc_r_a = 23
enc_r_b = 24
enc_l_a = 22
enc_l_b = 27

encoder_pins = [enc_l_a, enc_l_b, enc_r_a, enc_r_b]

encoder_freq = 10

motor_controller_pid = np.array([0.0, 0., 0.])
motor_control_freq = 100

# LEDS
num_leds = 3
led_default_brightness = 2  # Range from 0 to 31 (31 is really bright)
sclk = 8  # GPIO8 (hw pin 24) is hooked up to the SCLK pin
mosi = 26  # GPIO26 (hw pin 37) is hooked up to the MOSI pin

######################### PLANNER CONTROLLER PARAMS ################################
wall_transparency_threshold = 0.3
distance_to_cell_center_for_replan = 0.02

controller_steering_coeff = 1.5
controller_drive_coeff = 1.5
controller_max_speed = 0.1  # m/s
controller_max_turn = 1.  # r/s
controller_max_angle = np.pi / 6

######################### ESTIMATION PARAMS ################################
num_particles = 20

lidar_max_dist = 0.255  # the distance the lidars return when they are bad
lidar_sus_dist = 0.22  # the distance above which we don't really trust the lidars

maze_increment = 0.2
maze_decrement = 0.1

maze_backup_path = '/home/pi/ros_catkin_ws/src/pacmouse_pkg/src/utils/maze_backups'
