import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from pacmouse_pkg.src.estimation_control.dynamics import motion_model


cell_size = 0.089 # 3.5 inches in meters
freq = 10

class PacSim:
    def __init__(self):
        self.real_state = np.zeros(6)
        self.est_state = np.zeros(6)
        self.cmd = np.zeros(2)

        self.dt = 1./freq

        self.u_sigma = np.array([.002,.002, np.radians(2), 1e-4, 1e-4, 1e-4])
        self.forward_increment = 40.
        self.steer_increment = 40.

    def update(self):
        # udpate the real robot's position
        u_mu = motion_model(self.real_state, self.cmd, self.dt)
        self.real_state += np.random.normal(u_mu, self.u_sigma)

        # estimate the robots position
        u_mu = motion_model(self.est_state, self.cmd, self.dt)
        self.est_state += u_mu

        self.update_with_imu()
        self.update_with_cv()

    def update_with_imu(self):
        # read the IMU sensor data. this will hopefully be normally distributed around
        # the actual robot's position?
        self.est_state[2] = np.random.normal(self.real_state[2], self.u_sigma[2])

    def update_with_cv(self):
        # snap the estimated robot position into the cell that the real robot is in
        # we know which cell the real robot is in because that come from the pacbot CV
        real_cell = self.request_cell(self.real_state)
        est_cell = self.request_cell(self.est_state)

        print 'Correct cell' if (real_cell == est_cell).all() else 'Snapping!'


        if real_cell[0] > est_cell[0]:
            self.est_state[0] = real_cell[0]*cell_size
        elif real_cell[0] < est_cell[0]:
            self.est_state[0] = (real_cell[0]+1)*cell_size

        if real_cell[1] > est_cell[1]:
            self.est_state[1] = real_cell[1]*cell_size
        elif real_cell[1] < est_cell[1]:
            self.est_state[1] = (real_cell[1]+1)*cell_size

    def request_cell(self, x):
        # this emulates the data we will get from the pacbot computer vision
        return np.floor(x[:2]/cell_size)

    def draw_bot(self, plt, pose, color, alpha=1, size=0.03):
        arrow = mpatches.Arrow(pose[0], pose[1], size*np.cos(pose[2]), size*np.sin(pose[2]),
                               color=color, width=size/2, alpha=alpha)
        plt.add_patch(arrow)

    def animate_plot(self, i):
        self.update()
        ax1.clear()
        plt.xlim(-0.5, 0.5)
        plt.ylim(-0.5, 0.5)
        plt.gca().set_aspect('equal', adjustable='box')
        
        self.draw_bot(ax1, self.real_state, 'g')
        self.draw_bot(ax1, self.est_state, 'r')

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

if __name__ == '__main__':
    sim = PacSim()

    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    num_iterations = 10000
    animation = animation.FuncAnimation(fig, sim.animate_plot, frames=num_iterations, repeat=False, interval=sim.dt*1000)
    cid = fig.canvas.mpl_connect('key_press_event', sim.on_key_press)
    plt.show()