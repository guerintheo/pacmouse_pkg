import sys, os
sys.path.insert(0, os.path.abspath(".."))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from estimation import estimate_lidar_returns, maze_to_segment_list, plot_segment_list
from ParticleFilter import particle_filter_update
from map import Maze
import params as p

def gaussian_2d(self, mu, sigma, x, y):
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


class MazeParticleFilterTest:
    def __init__(self):
        self.maze = Maze(5,5)
        self.segment_list = maze_to_segment_list(self.maze)
        self.pose = np.array([0.084, 0.084, np.pi/4]) # x, y, theta
        self.Z = estimate_lidar_returns(self.pose, self.maze)
        self.num_particles = 100

        # create a bunch of particles in different positions and rotations
        self.particles = np.zeros([self.num_particles, 3])
        self.particles[:,0] = np.random.uniform(low=0, high=self.maze.width*p.maze_inner_size, size=self.num_particles)
        self.particles[:,1] = np.random.uniform(low=0, high=self.maze.height*p.maze_inner_size, size=self.num_particles)
        self.particles[:,2] = np.random.uniform(low=0, high=np.pi*2, size=self.num_particles)

        self.u_mu = np.array([0,0,0]) # assume the bot doesn't move
        self.u_sigma = np.array([.01,.01,.01])

    def obs_func(self, Z, x):
        z_exp = estimate_lidar_returns(x, self.maze)
        return np.sum(np.exp(-(100*(z_exp - Z))**2))

    def update(self):
        self.particles = particle_filter_update(self.particles, self.u_mu, self.u_sigma, self.Z, self.obs_func)

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        plot_segment_list(ax1, self.segment_list)
        ax1.scatter(self.particles[:,0], self.particles[:,1], color='r', alpha=.2)
        ax1.scatter(self.pose[0], self.pose[1], color='b')

if __name__ == "__main__":
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    # pf = SimpleParticleFilterTest()
    pf = MazeParticleFilterTest()
    num_iterations = 80
    animation = animation.FuncAnimation(fig, pf.animate_plot, frames=num_iterations, repeat=False, interval=10)
    plt.show()
