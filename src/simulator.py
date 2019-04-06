import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

from pacmouse_pkg.src.estimation_control.sensor_model import *
from pacmouse_pkg.src.estimation_control.estimation import Estimator
from pacmouse_pkg.src.estimation_control.dynamics import motion_model, inverse_motion_model
from pacmouse_pkg.src.estimation_control.control import step, get_sp
from pacmouse_pkg.src.utils.maze import Maze, Maze2
from pacmouse_pkg.src.utils.math_utils import rotate_2d, rotation_matrix_2d
import pacmouse_pkg.src.params as p
from pacmouse_pkg.src.estimation_control.planner import Planner


class Simulator:

    def __init__(self, use_macaroni=False):
        # build a maze and set the target sell
        self.maze = Maze(6,6)
        self.maze.build_wall_matrices()
        self.maze.build_segment_list()
        self.target_cell = [self.maze.width-1, self.maze.height-1]
        self.use_macaroni = use_macaroni

        # specify the initial state
        self.real_bot_state = np.array([0.5*p.maze_cell_size, 0.5*p.maze_cell_size, np.pi/2,0,0,0])

        # a nosie model for when the robot moves (should be the same as the estimator)
        # NOTE(izzy): this sigma should be estimated by the dyanmics model somehow???
        # We might have to collect mocap data in order to get this
        self.u_sigma = np.array([.002,.002, np.radians(2), 1e-4, 1e-4, 1e-4])
        # noise to add to simulated sensor data
        self.lidar_sigma = 0.005
        self.encoder_sigma = 5

        self.estimator = Estimator(self.real_bot_state, num_particles=50) # initialize the state estimator
        self.estimator.set_maze(self.maze)              # pass it the maze
        self.estimator.u_sigma = self.u_sigma           # and the noise model

        self.dt = 0.05
        
        if self.use_macaroni:
            # Create a macaroni plan once
            self.planner = Planner(self.maze)
            # Find path from bottom left corner to top right corner of maze
            target_cell_indices = self.maze.get_path([0, 0], [self.maze.width - 1, self.maze.height - 1])
            target_cells = []
            for i in target_cell_indices:
                col_row = list(self.maze.index_to_xy(i))
                target_cells.append([col_row[1], col_row[0]])  # [row, col]
            start_pose = [self.estimator.state[0] - 0.01, self.estimator.state[1] - 0.01, self.estimator.state[2]]
            self.planner.update_plan(target_cells, start_pose=start_pose)
            #self.planner.update_plan(target_cells)
        
    def update(self):
        # run the planner and controller to get commands (using the estimated state)
        if self.use_macaroni:
            curr_pose_estimate = [self.estimator.state[0], self.estimator.state[1], self.estimator.state[2]]
            plan_time_of_curr_pose = self.planner.get_t_on_path(curr_pose_estimate)
            set_point_lookahead_time = 0.3
            plan_time_of_set_point = plan_time_of_curr_pose + set_point_lookahead_time
            self.set_point = self.planner.get_reference_pose_from_plan_by_time(plan_time_of_set_point)
        else:
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
        lidars = estimate_lidar_returns_multi(self.real_bot_state[None,:3], self.maze) + np.random.normal(0, self.lidar_sigma, 6)
        encoders = cmd + np.random.normal(0, self.encoder_sigma, size=2)

        return lidars, encoders

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        plt.xlim(0, self.maze.width * p.maze_cell_size)
        plt.ylim(0, self.maze.height * p.maze_cell_size)
        plt.gca().set_aspect('equal', adjustable='box')
        self.maze.plot(plt)
        for particles in self.estimator.pf.particles: self.draw_bot(ax1, particles, 'r', 0.2)   # draw the particles
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
        # plt.scatter(lidar_global_xy[:,0], lidar_global_xy[:,1], color=color, alpha=alpha)
        
    def on_key_press(self, event):
        pass


class DrivingSimulator:

    def __init__(self):
        # build a maze and set the target sell
        self.maze_size = (8,8)
        self.real_maze = Maze2(*self.maze_size)

        temp_maze = Maze(*self.maze_size)
        temp_maze.build_wall_matrices()
        print temp_maze
        self.real_maze.v_walls = 1 -temp_maze.v_walls
        self.real_maze.h_walls = 1 -temp_maze.h_walls

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
        self.estimator.set_maze(self.real_maze)              # pass it the maze
        self.estimator.u_sigma = self.u_sigma           # and the noise model

        self.estimated_maze = Maze2(*self.maze_size)
        self.estimated_maze.build_segment_list()

        self.cmd = np.zeros(2)
        self.forward_increment = 40.
        self.steer_increment = 40.
        self.dt = 0.2

    def update_estimated_maze(self):
        pose = self.estimator.state[:3]

        decrement_amount = 0.12
        increment_amount = 0.1
        update_walls(pose, self.lidars, self.estimated_maze, decrement_amount, increment_amount)

        self.estimated_maze.build_segment_list()

    def update(self):
        # run the simulator to update the "real" robot
        Z = self.simulate(self.cmd)
        self.lidars, self.encoders = Z

        self.update_estimated_maze()

        # and update the estimator
        self.estimator.update(Z, self.dt)

    def simulate(self, cmd):
        # update the actual robot according to the commands (with noise)
        u_mu = motion_model(self.real_bot_state, cmd, self.dt)
        self.real_bot_state += np.random.normal(u_mu, self.u_sigma)

        # get the sensor data (with noise)
        lidars = estimate_lidar_returns(self.real_bot_state[:3], self.real_maze) + np.random.normal(0, self.lidar_sigma, 6)
        encoders = cmd + np.random.normal(0, self.encoder_sigma, size=2)

        return lidars, encoders

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        plt.xlim(0, self.real_maze.width * p.maze_cell_size)
        plt.ylim(0, self.real_maze.height * p.maze_cell_size)
        plt.gca().set_aspect('equal', adjustable='box')
        self.estimated_maze.plot(plt)
        for particle in self.estimator.pf.particles: self.draw_bot(ax1, particle, 'r', 0.2)   # draw the particles
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
        lidar_global_xy = lidar_end_points(pose, self.lidars)
        plt.scatter(lidar_global_xy[:,0], lidar_global_xy[:,1], color=color, alpha=alpha)

    def on_key_press(self, event):
        """
        Matplotlib keypress event handler that adjusts the angular velocities of
        the left and right motors of the robot.
        """
        if (event.key == "left"):
            self.cmd += np.array([-1, 1]) * self.steer_increment
        elif (event.key == "right"):
            self.cmd += np.array([1, -1]) * self.steer_increment
        elif (event.key == "up"):
            self.cmd += self.forward_increment
        elif (event.key == "down"):
            self.cmd -= self.forward_increment


class FullSimulator:

    def __init__(self):
        # build a maze and set the target sell
        self.maze_size = (8,8)
        self.real_maze = Maze2(*self.maze_size)

        temp_maze = Maze(*self.maze_size)
        temp_maze.build_wall_matrices()
        print temp_maze
        self.real_maze.v_walls = 1 - temp_maze.v_walls
        self.real_maze.h_walls = 1 - temp_maze.h_walls

        # specify the initial state
        self.real_bot_state = np.array([0.5*p.maze_cell_size, 0.5*p.maze_cell_size, np.pi/2,0,0,0])

        self.estimated_maze = Maze2(*self.maze_size)
        self.estimated_maze.v_walls[1:-1, :] = 0.5 # set all the confidences to one half
        self.estimated_maze.h_walls[:, 1:-1] = 0.5

        # a nosie model for when the robot moves (should be the same as the estimator)
        # NOTE(izzy): this sigma should be estimated by the dyanmics model somehow???
        # We might have to collect mocap data in order to get this
        self.u_sigma = np.array([.002,.002, np.radians(2), 1e-4, 1e-4, 1e-4])
        # noise to add to simulated sensor data
        self.lidar_sigma = 0.005
        self.encoder_sigma = 5

        self.estimator = Estimator(self.real_bot_state)                         # initialize the state estimator
        self.estimator.set_maze(self.estimated_maze, obs_func=lidar_observation_function_gaussian)   # pass it the maze
        self.estimator.u_sigma = self.u_sigma           # and the noise model

        self.cmd = np.zeros(2)
        self.forward_increment = 40.
        self.steer_increment = 40.
        self.dt = 0.2

    def update_estimated_maze(self):
        pose = self.estimator.state[:3]

        decrement_amount = 0.06
        increment_amount = 0.05
        update_walls(pose, self.lidars, self.estimated_maze, decrement_amount, increment_amount)

        self.estimated_maze.build_segment_list()
        self.estimator.set_maze(self.estimated_maze, obs_func=lidar_observation_function_gaussian)

    def update(self):
        # run the simulator to update the "real" robot
        Z = self.simulate(self.cmd)
        self.lidars, self.encoders = Z

        self.update_estimated_maze()

        # and update the estimator
        self.estimator.update(Z, self.dt)

    def simulate(self, cmd):
        # update the actual robot according to the commands (with noise)
        u_mu = motion_model(self.real_bot_state, cmd, self.dt)
        self.real_bot_state += np.random.normal(u_mu, self.u_sigma)

        # get the sensor data (with noise)
        lidars, _ = estimate_lidar_returns(self.real_bot_state[:3], self.real_maze) + np.random.normal(0, self.lidar_sigma, 6)
        encoders = cmd + np.random.normal(0, self.encoder_sigma, size=2)

        return lidars, encoders

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        plt.xlim(0, self.real_maze.width * p.maze_cell_size)
        plt.ylim(0, self.real_maze.height * p.maze_cell_size)
        plt.gca().set_aspect('equal', adjustable='box')
        self.estimated_maze.plot(plt)
        for particle in self.estimator.pf.particles: self.draw_bot(ax1, particle, 'r', 0.2)   # draw the particles
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
        lidar_global_xy = lidar_end_points(pose, self.lidars)
        plt.scatter(lidar_global_xy[:,0], lidar_global_xy[:,1], color=color, alpha=alpha)

    def on_key_press(self, event):
        """
        Matplotlib keypress event handler that adjusts the angular velocities of
        the left and right motors of the robot.
        """
        if (event.key == "left"):
            self.cmd += np.array([-1, 1]) * self.steer_increment
        elif (event.key == "right"):
            self.cmd += np.array([1, -1]) * self.steer_increment
        elif (event.key == "up"):
            self.cmd += self.forward_increment
        elif (event.key == "down"):
            self.cmd -= self.forward_increment

if __name__ == "__main__":
    drive = (len(sys.argv) > 1)

    sim = Simulator()

    if drive and sys.argv[1] == 'drive':
        sim = DrivingSimulator()
    elif drive and sys.argv[1] == 'full':
        sim = FullSimulator()
    elif drive and sys.argv[1] == 'macaroni':
        sim = Simulator(use_macaroni=True)

    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    num_iterations = 10000
    animation = animation.FuncAnimation(fig, sim.animate_plot, frames=num_iterations, repeat=False, interval=sim.dt*1000)
    if drive: cid = fig.canvas.mpl_connect('key_press_event', sim.on_key_press)
    plt.show()
