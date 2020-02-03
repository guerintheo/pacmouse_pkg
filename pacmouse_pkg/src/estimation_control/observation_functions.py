import numpy as np

from pacmouse_pkg.src.utils.maze import Maze, Maze2
import pacmouse_pkg.src.params as p
from pacmouse_pkg.src.utils.math_utils import *
from pacmouse_pkg.src.estimation_control.sensor_model import estimate_lidar_returns_multi

def lidar_observation_function_hyperbolic(Z, x, maze):
    """Computes a likelihood given sensor data and a particle position. A higher
    number means x is more likely to produce an observation similar to Z

    Args:
        Z (1d numpy array): a 6-vector of lidar measurements
        x (1d numpy array): a 6-vector of state (of a particle)
        maze (Maze): the maze

    # Returns:
        float: how likely that particle is based on agreement the sensor data, Z
    """
    z_exp = estimate_lidar_returns_multi(x[:,None], maze)[0]
    return np.prod(1/(np.abs(z_exp - Z) + 1e-5))

def lidar_observation_function_hyperbolic_multi(Z, xs, maze):
    """Computes a likelihood given sensor data and a particle position. A higher
    number means x is more likely to produce an observation similar to Z

    Args:
        Z (1d numpy array): a 6-vector of lidar measurements
        x (2d numpy array): an n by 6 of state (of a particle)
        maze (Maze): the maze

    Returns:
        1d numpy array: how likely each particle is based on agreement the sensor data, Z
    """
    z_exp = estimate_lidar_returns_multi(xs, maze)
    return np.prod(1/(np.abs(z_exp - Z) + 1e-5), axis=1)

def lidar_observation_function_gaussian(Z, x, maze):
    z_exp, z_conf = estimate_lidar_returns(x, maze)

    # NOTE(izzy): we take 1-z_conf because z_conf represents how likely there is to be
    # wall at the point where the lidar hits, and we want the variance to be lower if
    # we are more certain that there is a wall at that position.

    # NOTE(aaron): we may need to scale and offset z_conf to avoid zero variance
    # NOTE(aaron): z_conf should never be smaller than the p.wall_transparency_threshold arg
    # to the estimate_lidar_returns model
    base_variance = 0.01
    return np.prod(gaussian(z_exp, Z, base_variance + (1.-z_conf)/100.))

def lidar_observation_function_gaussian_multi(Z, xs, maze):
    z_exp, z_conf = estimate_lidar_returns_multi(xs, maze, return_confidences=True)

    # NOTE(izzy): we take 1-z_conf because z_conf represents how likely there is to be
    # wall at the point where the lidar hits, and we want the variance to be lower if
    # we are more certain that there is a wall at that position.

    # NOTE(aaron): we may need to scale and offset z_conf to avoid zero variance
    # NOTE(aaron): z_conf should never be smaller than the p.wall_transparency_threshold arg
    # to the estimate_lidar_returns model
    base_variance = 0.01
    eps = 1e-8
    # return np.prod(gaussian(z_exp, Z[None,:], base_variance + (1.-z_conf)/100.)+1e-5, axis=1)
    return np.prod(gaussian(z_exp, Z[None,:], base_variance / (z_conf+eps)), axis=1) + eps