import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

from pacmouse_pkg.src.estimation_control.sensor_model import estimate_lidar_returns, which_walls
from pacmouse_pkg.src.estimation_control.estimation import Estimator
from pacmouse_pkg.src.estimation_control.dynamics import motion_model, inverse_motion_model
from pacmouse_pkg.src.estimation_control.control import step, get_sp
from pacmouse_pkg.src.utils.maze import Maze
from pacmouse_pkg.src.utils.math_utils import rotate_2d, rotation_matrix_2d
import pacmouse_pkg.src.params as p


class Simulator:

    def __init__(self):
        # build a maze and set the target sell
        self.maze = Maze(6,6)
        self.maze.build_wall_matrices()
        self.maze.build_segment_list()
        self.target_cell = [self.maze.width-1, self.maze.height-1]

        # specify the initial state
        self.real_bot_state = np.array([0.5*p.maze_cell_size, 0.5*p.maze_cell_size, np.pi/2,0,0,0])

        # a nosie model for when the robot moves (should be the same as the estimator)
        # NOTE(izzy): this sigma should be estimated by the dyanmics model somehow???
        # We might have to collect mocap data in order to get this
        self.u_sigma = np.array([.002,.002, np.radians(2), 1e-4, 1e-4, 1e-4])
        # noise to add to simulated sensor data
        self.lidar_sigma = 0.005
        self.encoder_sigma = 5

        self.estimator = Estimator(self.real_bot_state) # initialize the state estimator
        self.estimator.set_maze(self.maze)              # pass it the maze
        self.estimator.u_sigma = self.u_sigma           # and the noise model

        self.dt = 0.2

    def update(self):
        # run the planner and controller to get commands (using the estimated state)
        self.set_point = get_sp(self.estimator.state, self.maze, self.target_cell)
        cmd = inverse_motion_model(step(self.estimator.state, self.set_point))

        # run the simulator to update the "real" robot
        Z = self.simulate(cmd)

        # and update the estimator
        self.estimator.update(Z, self.dt)

    def simulate(self, cmd):
        # update the actual robot according to the commands (with noise)
        u_mu = motion_model(self.real_bot_state, cmd, self.dt)
        self.real_bot_state += np.random.normal(u_mu, self.u_sigma)

        # get the sensor data (with noise)
        self.lidars = estimate_lidar_returns(self.real_bot_state[:3], self.maze) + np.random.normal(0, self.lidar_sigma, 6)
        encoders = cmd + np.random.normal(0, self.encoder_sigma, size=2)

        return self.lidars, encoders

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        self.maze.plot(plt)
        for p in self.estimator.pf.particles: self.draw_bot(ax1, p, 'r', 0.2)   # draw the particles
        self.draw_outer_chassis(ax1, self.estimator.state[:3], 'g', 0.5)        # draw the estimated bot
        self.draw_outer_chassis(ax1, self.real_bot_state[:3], 'b', 0.5)         # draw the real bot

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

        # draw the positions of the lidars
        # R = rotation_matrix_2d(pose[2])
        # lidar_global_xy = pose[None, :2] + np.dot(R, p.lidar_transforms[:, :2].T).T
        lidar_global_xy = which_walls(pose ,self.lidars)
        plt.scatter(lidar_global_xy[:,0], lidar_global_xy[:,1], color=color, alpha=alpha)


if __name__ == "__main__":
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    sim = Simulator()
    num_iterations = 10000
    animation = animation.FuncAnimation(fig, sim.animate_plot, frames=num_iterations, repeat=False, interval=10)
    plt.show()
