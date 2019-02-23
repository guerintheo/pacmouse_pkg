#!/usr/bin/env python
import os,sys
sys.path.insert(1, os.path.join(sys.path[0], '..'))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from sensor_model import estimate_lidar_returns, maze_to_segment_list, plot_segment_list, lidar_observation_function
from particle_filter import particle_filter_update, ParticleFilter
from dynamics import motion_model
from control import step, get_sp, mix
from maze import Maze
import params as p

def gaussian_2d(mu, sigma, x, y):
    X = np.array([x,y])
    return np.exp(-1/2 * np.dot((X - mu).transpose(),np.dot(np.linalg.inv(sigma),(X - mu))))/\
           (np.sqrt((2*np.pi)**2 * np.linalg.det(sigma)))


class SimpleParticleFilterTest(object):
    def __init__(self):
        self.mu = np.array([0,0])
        self.sigma = np.array([[5,0],[0,5]])
        xpoints = np.linspace(-5,5,100)
        ypoints = np.linspace(-5,5,100)
        self.zpoints = np.zeros((len(ypoints),len(xpoints)))
        for ix in range(len(xpoints)):
            for iy in range(len(ypoints)):
                x = xpoints[ix]
                y = ypoints[iy]
                self.zpoints[iy][ix] = gaussian_2d(self.mu, self.sigma, x, y)

        self.n_parts = 1000
        self.real_obj = np.array([-4,-4])
        self.particles = np.random.uniform(low=-5,high=5, size=(self.n_parts, 2))
        self.u_mu = np.array([.1,.1])
        self.u_sigma = np.array([.05,.05])

    def obs_func(self, Z, x):
        """ P(Z | x) """
        z_exp = gaussian_2d(self.mu, self.sigma, x[0], x[1])
        return np.exp(-(100*(z_exp - Z))**2)

    def animate_plot(self, i):

        self.real_obj = self.real_obj + np.random.normal(self.u_mu, self.u_sigma)
        #Z = gaussian_2d(self.mu, self.sigma, self.real_obj[0], self.real_obj[1]) + np.random.normal(0, .01)
        Z = gaussian_2d(self.mu, self.sigma, self.real_obj[0], self.real_obj[1])

        self.particles = particle_filter_update(self.particles, self.u_mu, self.u_sigma, Z, self.obs_func)
        ax1.clear()
        ax1.imshow(self.zpoints,extent=[-5,5,-5,5])
        ax1.scatter(self.particles[:,0], self.particles[:,1], color='r', alpha=.2)
        ax1.scatter(self.real_obj[0], self.real_obj[1], color='b')


class SimpleMazeParticleFilterTest:
    def __init__(self):
        self.maze = Maze(4,4)
        self.segment_list = maze_to_segment_list(self.maze)
        self.pose = np.array([1.5*p.maze_cell_size, 1.5*p.maze_cell_size, np.pi/4]) # x, y, theta

        self.num_particles = 20

        # create a bunch of particles in different positions and rotations
        self.particles = np.zeros([self.num_particles, 3])
        # self.particles[:,0] = np.random.uniform(low=0, high=self.maze.width*p.maze_cell_size, size=self.num_particles)
        # self.particles[:,1] = np.random.uniform(low=0, high=self.maze.height*p.maze_cell_size, size=self.num_particles)
        # self.particles[:,2] = np.random.uniform(low=0, high=np.pi*2, size=self.num_particles)
        self.particles[:,:] = self.pose[None,:]

        self.u_mu = np.array([0, 0, 0.05]) # assume the bot rotates in place
        self.u_sigma = np.array([.005,.005, 0.05]) # we lock the rotation because we have IMU

        self.lidar_sigma = 0.001 # standard deviation

    def obs_func(self, Z, x):
        z_exp = estimate_lidar_returns(x, self.maze)
        # NOTE(izzy): experimenting with different loss functions
        # return np.prod(np.exp(-np.abs(z_exp - Z)))            # exponential
        # return np.sum(-np.abs(z_exp - Z))                     # log exponential
        return np.prod(1/(np.abs(z_exp - Z) + 1e-5))          # inverse
        # return np.sum(np.log(1/(np.abs(z_exp - Z) + 1e-5)))   # log inverse
        # TODO: add normal (gaussian) loss (mu is Z)

    def update(self):
        self.pose[2] += 0.05 # rotate the robot
        # get the sensor data
        self.Z = estimate_lidar_returns(self.pose, self.maze) + np.random.normal(0, self.lidar_sigma, 6)

        self.particles = particle_filter_update(self.particles, self.u_mu, self.u_sigma, self.Z, self.obs_func)

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        plot_segment_list(ax1, self.segment_list)
        for p in self.particles:
            self.draw_bot(ax1, p, 'r', 0.2)
        self.draw_bot(ax1, self.pose, 'b', 1)

    def draw_bot(self, plt, pose, color, alpha, size=0.03):
        arrow = mpatches.Arrow(pose[0], pose[1], size*np.cos(pose[2]), size*np.sin(pose[2]),
                               color=color, width=size/2, alpha=alpha)
        plt.add_patch(arrow)


class StateEstimatorParticleFilter(object):
    
    def __init__(self):
        self.n_dims = 6
        self.real_obj_state = np.array([-4.0, -4.0, np.pi/4, 0.0, 0.0, 0.0])
        self.initialize_background_gaussian()
        
        self.n_parts = 2000
        self.particles = np.random.uniform(low=-5,high=5, size=(self.n_parts, self.n_dims))
        # Noise due to motion
        self.u_sigma = np.array([.008, .008, .008, .008, .008, .008])
                
        # Angular velocity control inputs self.omega_l and self.omega_r
        self.omega_l = 1000
        self.omega_r = 1000
        self.input_increment = 30
        
    def initialize_background_gaussian(self):
        self.mu = np.array([0, 0]) #  mean of background Gaussian
        self.sigma = np.array([[5,0],[0,5]]) # covariance of background Gaussian
        xpoints = np.linspace(-5,5,100)
        ypoints = np.linspace(-5,5,100)
        self.zpoints = np.zeros((len(ypoints),len(xpoints)))
        for ix in range(len(xpoints)):
            for iy in range(len(ypoints)):
                x = xpoints[ix]
                y = ypoints[iy]
                self.zpoints[iy][ix] = gaussian_2d(self.mu, self.sigma, x, y)

    def obs_func(self, Z, x):
        """ P(Z | x) """
        z_exp = gaussian_2d(self.mu, self.sigma, x[0], x[1])
        return np.exp(-(100*(z_exp - Z))**2)

    def animate_plot(self, i, pf):
        mean_change = motion_model(self.real_obj_state, np.array([self.omega_l, self.omega_r]), 0.2)
        # Add motion and noise to real robot
        self.real_obj_state += np.random.normal(mean_change, self.u_sigma)
        
        #Z = gaussian_2d(self.mu, self.sigma, self.real_obj_state[0], self.real_obj_state[1]) + np.random.normal(0, .01)
        Z = gaussian_2d(self.mu, self.sigma, self.real_obj_state[0], self.real_obj_state[1])

        self.particles = particle_filter_update(self.particles, mean_change, self.u_sigma, Z, self.obs_func)
        ax1.clear()
        ax1.set_xlim(-5, 5)
        ax1.set_ylim(-5, 5)
        ax1.imshow(self.zpoints,extent=[-5,5,-5,5])
        ax1.scatter(self.particles[:, 0], self.particles[:, 1], color='r', alpha=.2)
        #ax1.scatter(self.real_obj_state[0], self.real_obj_state[1], color='b')
        arrow = mpatches.Arrow(self.real_obj_state[0], self.real_obj_state[1], 0.4*np.cos(self.real_obj_state[2]), 0.4*np.sin(self.real_obj_state[2]), color="blue", width=0.3)
        ax1.add_patch(arrow)
        
    def on_key_press(self, event):
        """
        Matplotlib keypress event handler that adjusts the angular velocities of
        the left and right motors of the robot.
        """
        if (event.key == "left"):
            self.omega_r += self.input_increment
            self.omega_l -= self.input_increment
        elif (event.key == "right"):
            self.omega_l += self.input_increment
            self.omega_r -= self.input_increment
        elif (event.key == "up"):
            self.omega_l += self.input_increment
            self.omega_r += self.input_increment
        elif (event.key == "down"):
            self.omega_l -= self.input_increment
            self.omega_r -= self.input_increment



if __name__ == "__main__":
    #### To run MazeParticleFilter or SimpleParticelFilter ####
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    # pf = SimpleParticleFilterTest()
    # pf = SimpleMazeParticleFilterTest()
    pf = DrivingMazeParticleFilterTest()
    num_iterations = 10000
    animation = animation.FuncAnimation(fig, pf.animate_plot, frames=num_iterations, repeat=False, interval=10)
    # cid = fig.canvas.mpl_connect('key_press_event', pf.on_key_press)
    plt.show()

    #### To run StateEstimatorParticleFilter ####
    # fig = plt.figure(figsize=(14, 7))
    # ax1 = fig.add_subplot(1,1,1)
    # pf = StateEstimatorParticleFilter()
    # animation = animation.FuncAnimation(fig, pf.animate_plot, fargs=(pf,), interval=10)
    # cid = fig.canvas.mpl_connect('key_press_event', pf.on_key_press)
    # plt.show()
