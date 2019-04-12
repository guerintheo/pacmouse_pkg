import numpy as np
from copy import deepcopy
import pacmouse_pkg.src.params as p
from pacmouse_pkg.src.estimation_control.dynamics import motion_model
from pacmouse_pkg.src.estimation_control.particle_filter import ParticleFilter
from pacmouse_pkg.src.estimation_control.sensor_model import *

class Estimator:

    def __init__(self, state, num_particles=20):
        self.state = deepcopy(state[:])

        # NOTE(izzy): this sigma should be estimated by the dyanmics model somehow???
        # We might have to collect mocap data in order to get this
        self.u_sigma = np.array([.002,.002, 0.0, 0, 0, 0])

        self.num_particles = num_particles
        particles = np.zeros([self.num_particles, 3])
        particles[:,:] = self.state[None,:3] # set the particles to be at the same position as the state
        self.pf = ParticleFilter(particles)
        self.pf.parallelized = True # compute the observation function simultaneously for all particles

    def set_maze(self, maze, obs_func=lidar_observation_function_hyperbolic_multi):
        self.lidar_obs_func = lambda Z, x: obs_func(Z, x, maze)

    def update(self, Z, dt):
        # NOTE(izzy): this is not the right way to do this, I'm just filling in the class so
        # we can get the architecture up and running. In the future this should be in a kalman
        # filter.
        lidars, encoders, imu = Z

        # right now I'm using the same dynamics from the motion_model
        u_mu = motion_model(self.state, encoders, dt)

        # lock the change in rotation to the imu orientation (this is meh, but shred it)
        u_mu[2] = imu - self.state[2]

        # update the particle filter
        self.pf.update(u_mu[:3], self.u_sigma[:3], lidars, self.lidar_obs_func)
        # TODO: Consider whether to model each particle as a 3-vector or a
        # 6-vector. If modeled as 6-vectors, then we should probably use the
        # complete motion model on each of the particles

        # use an exponential moving average to blend the best particle with the current state
        # 1 means taking the best particle. 0 means completely ignoring the particle filter
        pf_alpha = 1
        best_particle = self.pf.particles[np.argmax(self.pf.likelihoods)]
        self.state[:3] = best_particle * pf_alpha + self.state[:3] * (1-pf_alpha)
