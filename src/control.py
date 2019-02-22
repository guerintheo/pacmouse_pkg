import numpy as np

def step(x, sp):
	steering_coeff = 100.
	drive_coeff    = 300.

	e_x, e_y = sp[:2] - x[:2] 					# the position difference
	e_t = x[2] - np.arctan2(e_y, e_x) 			# the angular error to point at the desired position
	d = np.sqrt(e_x**2 + e_y**2)				# the distance to the desired position
	facing_target = (np.pi - np.abs(e_t))/np.pi # 1 when facing the target. 0 when facing away from target

	steering_command = np.array([e_t, -e_t]) * steering_coeff		# turn to face the target
	drive_command = np.ones(2) * d * facing_target * drive_coeff	# drive forward faster when facing the target

	return steering_command + drive_command