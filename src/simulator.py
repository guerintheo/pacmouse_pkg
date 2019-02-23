import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from sensor_model import *
from particle_filter import ParticleFilter
from dynamics import motion_model
from control import step, get_sp, mix
from maze import Maze
from util import rotate_2d
import params as p


class DrivingMazeParticleFilterTest:

    def __init__(self):
        self.maze = Maze(6,6)
        self.maze.build_wall_matrices()
        self.segment_list = maze_to_segment_list(self.maze)
        self.state = np.array([0.5*p.maze_inner_size, 0.5*p.maze_inner_size, np.pi/2,0,0,0]) # x, y, theta, dx, dy, psi
        self.target_cell = [self.maze.width-1, self.maze.height-1]

        num_particles = 20
        particles = np.zeros([num_particles, 3])
        particles[:,:] = self.state[None,:3] # set the particles to be at the same position as the state
        self.pf = ParticleFilter(particles)

        self.u_mu = np.array([0, 0, 0, 0, 0, 0]) # assume the bot rotates in place
        self.u_sigma = np.array([.001,.001, 0.05, 1e-4, 1e-4, 1e-4]) # we lock the rotation because we have IMU

        self.lidar_sigma = 0.005 # standard deviation
        
        # Angular velocity control inputs self.omega_l and self.omega_r
        self.omega_l = 100
        self.omega_r = 100

    def obs_func(self, Z, x):
        return lidar_observation_function(Z, x, self.maze)

    def update(self):
        # Add motion and noise to real robot
        self.set_point = get_sp(self.state, self.maze, self.target_cell)
        (self.omega_l, self.omega_r) = mix(step(self.state, self.set_point))

        # TODO: Consider whether to model each particle as a 3-vector or a
        # 6-vector. If modeled as 6-vectors, then we should probably use the
        # complete motion model on each of the particles
        self.u_mu = motion_model(self.state, np.array([self.omega_l, self.omega_r]), 0.2)
        self.state += np.random.normal(self.u_mu, self.u_sigma)  

        # get the sensor data
        self.Z = estimate_lidar_returns(self.state[:3], self.maze) + np.random.normal(0, self.lidar_sigma, 6)

        self.pf.update(self.u_mu[:3], self.u_sigma[:3], self.Z, self.obs_func)

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        plot_segment_list(ax1, self.segment_list)
        for p in self.pf.particles:
            self.draw_bot(ax1, p, 'r', 0.2)
        self.draw_outer_chassis(ax1, self.state[:3], 'b', 1)

    def draw_bot(self, plt, pose, color, alpha, size=0.03):
        arrow = mpatches.Arrow(pose[0], pose[1], size*np.cos(pose[2]), size*np.sin(pose[2]),
                               color=color, width=size/2, alpha=alpha)
        plt.add_patch(arrow)

    def draw_outer_chassis(self, plt, pose, color, alpha):
        corners = np.array([[0,1,1,0], [0,0,1,1]]) - 0.5
        corners *= np.array([p.robot_length, p.robot_width])[:, None]
        corners = np.array([rotate_2d(c, pose[2]) for c in corners.T])
        corners += pose[None, :2]

        for i in range(4):
            j = (i+1)%4
            x1, y1 = corners[i]
            x2, y2 = corners[j]
            plt.plot((x1, x2), (y1, y2), color=color, alpha=alpha)


if __name__ == "__main__":
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    pf = DrivingMazeParticleFilterTest()
    num_iterations = 10000
    animation = animation.FuncAnimation(fig, pf.animate_plot, frames=num_iterations, repeat=False, interval=10)
    # cid = fig.canvas.mpl_connect('key_press_event', pf.on_key_press)
    plt.show()