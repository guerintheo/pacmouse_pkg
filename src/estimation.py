import numpy as np
import params as p
from copy import deepcopy
from dynamics import motion_model
from particle_filter import ParticleFilter
from sensor_model import lidar_observation_function

class Estimator:

    def __init__(self, state):
        self.state = deepcopy(state[:])

        # NOTE(izzy): this sigma should be estimated by the dyanmics model somehow???
        # We might have to collect mocap data in order to get this
        self.u_sigma = np.array([.001,.001, 0.05, 1e-4, 1e-4, 1e-4])

        num_particles = 20
        particles = np.zeros([num_particles, 3])
        particles[:,:] = self.state[None,:3] # set the particles to be at the same position as the state
        self.pf = ParticleFilter(particles)

    def set_maze(self, maze):
        self.lidar_obs_func = lambda Z, x: lidar_observation_function(Z, x, maze)

    def update(self, Z, dt):
        # NOTE(izzy): this is not the right way to do this, I'm just filling in the class so
        # we can get the architecture up and running. In the future this should be in a kalman
        # filter.
        lidars, encoders = Z

        # right now I'm using the same dynamics from the motion_model
        u_mu = motion_model(self.state, encoders, dt)
        self.state += u_mu

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