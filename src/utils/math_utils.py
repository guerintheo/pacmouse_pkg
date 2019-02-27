import numpy as np

def wrap(theta):
	"""Constrains rotation values from -pi to pi
	
	Args:
	    theta (float): angle in radians
	
	Returns:
	    float: angle in radians (-pi to pi)
	"""
	return (theta + np.pi) % (2*np.pi) - np.pi

def rotation_matrix_2d(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])

def rotate_2d(coord, theta):
    return np.dot(rotation_matrix_2d(theta), coord)

def rotate_2d_multiple(coords, thetas):
	Rs = np.moveaxis(rotation_matrix_2d(thetas), 2, 0)
	return np.einsum('ij,ikj->ik',coords,Rs)