import numpy as np
import pacmouse_pkg.src.params as p

def motion_model(x, u, dt):
    """
    Given the previous state (x) and control input (u), calculate the
    predicted change in state at the next time step.

    x: previous state: [x, y, psi, v_body, a_body, psi_dot]
    u: control input; NumPy array containing [angular_velocity_left_motor,
       angular_velocity_right_motor]
    dt: time step
    """
    controlled_vel_and_psi_dot = vel_and_psi_dot_from_wheel_vels(u)
    v = controlled_vel_and_psi_dot[0]
    psi_dot = controlled_vel_and_psi_dot[1]
    # TODO: Handle the psi_dot = 0 case perhaps a bit better, to avoid division by zero
    if psi_dot == 0:
        psi_dot = 1e-8

    psi = x[2]
    a = x[4]

    dpsi = psi_dot*dt
    dv = a*dt

    dx = 1.0/(psi_dot**2.0)*((v+dv)*psi_dot*np.sin(psi + dpsi) - v*psi_dot*np.sin(psi) + a*np.cos(psi + dpsi) - a*np.cos(psi))
    dy = 1.0/(psi_dot**2.0)*((-v-dv)*psi_dot*np.cos(psi + dpsi) + v*psi_dot*np.cos(psi) + a*np.sin(psi + dpsi) - a*np.sin(psi))

    change_in_state = np.array([dx, dy, dpsi, dv, 0.0, 0.0])
    return change_in_state

# constants used in the dynamics model for calculating velocities from wheel speeds
C_v = p.wheel_radius
C_psi_dot = (p.wheel_radius*p.wheel_dist_y)/(p.wheel_dist_x**2 + p.wheel_dist_y**2)

def vel_and_psi_dot_from_wheel_vels(u):
    """
    Use the robot kinematics model to compute the forward velocity and angular
    velocity of the robot given left and right motor angular velocities. See
    LaTeX document for the math.
    """
    v = p.wheel_radius/2.0*((u[0]+u[1]))
    psi_dot = (p.wheel_radius*p.wheel_dist_y)/(2*(p.wheel_dist_x**2 + p.wheel_dist_y**2))*((u[1]-u[0]))
    return np.array([v, psi_dot])

def motion_model2(x, u, dt):
	'''
	This is a simplified version of the above motion model using constant linear and angular velocities.
	Ultimately, this corresponds to time-parametrized motion along a circle or a straight line
	'''
	v 		= (u[0] + u[1])/2. * C_v 
	psi_dot = (u[1] - u[0])/2. * C_psi_dot

	if psi_dot == 0:
		dx = v * dt * np.cos(x[2])
		dy = v * dt * np.sin(x[2])
		dpsi = 0

		# center = x[:2]
		# angular_acceleration = 0

	else:
		dpsi = psi_dot*dt

		start_psi = x[2]
		end_psi = start_psi + dpsi

		# 2*pi*r / v 		= time to go around circle
		# 2*pi / psi_dot 	= time to go around circle
		# r = v/psi_dot

		# the radius of the turning circle
		signed_radius = v/psi_dot
		radius = np.abs(signed_radius)

		# center = signed_radius * np.array([-np.sin(psi), np.cos(psi)]) /+ x[:2] # the center of the turning circle
		# angular_acceleration = psi_dot**2 * radius

		dx = signed_radius * (np.sin(end_psi) - np.sin(start_psi))
		dy = signed_radius * (np.cos(start_psi) - np.cos(end_psi))

	change_in_state = np.zeros_like(x) # output state should be the same size as the input state
	change_in_state[:3] = [dx, dy, dpsi]
	return change_in_state

def inverse_motion_model(cmd):
    """ Given a desired forward velocity and turning rate, return the corresponding wheel speeds

    Args:
        cmd (1d numpy array): 2-vector of forward velocity in meters/second and
                              turning velocity in radians/second (positive is CCW)

    Returns:
        1d numpy array: 2-vector of [left, right] wheel speeds in radians/second
    """
    v, d_theta = cmd
    gvy = v * p.wheel_dist_y
    ry = p.wheel_radius * p.wheel_dist_y
    gtd = d_theta * (p.wheel_dist_x**2 + p.wheel_dist_y**2)

    return (gvy + np.array([-1,1]) * gtd)/ry
