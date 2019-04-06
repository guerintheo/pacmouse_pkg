# NOTE (Theo): matplotlib key presses work for me in python3 but not in python2
# NOTE (Theo): The MPC is far from complete
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

from pacmouse_pkg.src.estimation_control import dynamics

class MPC(object):
    
    def __init__(self):
        # Initial control inputs
        self.u = np.array([100., 100.]) # left and right motor angular velocities
        self.initialize_state(self.u)
        self.forward_increment = 300.
        self.steer_increment = 600.
        self.dt = 0.01 # time step in seconds
        self.state_labels = ['x','y','psi','v','a','psi_dot']
        
        ######### More MPC-specific stuff ############
        
        # Number of time steps into the future over which to optimize state
        self.prediction_horizon = 50
        # Number of time steps into the future over whicht to manipulate control inputs
        self.control_horizon = 10
        # Reference poses that we want to track
        self.initialize_reference_poses()

    def initialize_state(self, u_first):
        self.state = np.array([0.5,0.5,0,0,0,0], dtype='float64')
        self.pose = np.array([0.5,0.5,0], dtype='float64')
        self.state += dynamics.motion_model(self.state, u_first, 1)
        controlled_vel_and_psi_dot = dynamics.vel_and_psi_dot_from_wheel_vels(u_first)
        self.state[3] = controlled_vel_and_psi_dot[0]
        self.state[5] = controlled_vel_and_psi_dot[1]
        self.update_pose()
        
    def initialize_reference_poses(self):
        self.reference_poses = np.vstack((np.linspace(0.5, 0.598, num=self.prediction_horizon),
                                                 np.ones(self.prediction_horizon)*0.75,
                                                 np.zeros(self.prediction_horizon)))
        # print(self.reference_poses.shape)
        
    def start_sim(self):
        fig = plt.figure()
        self.ax1 = fig.add_subplot(1,1,1)
        num_iterations = 1000
        anim = animation.FuncAnimation(fig, self.animate_plot, frames=num_iterations, repeat=True, interval=self.dt*1000)
        cid = fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        plt.show()
        
    def animate_plot(self, i):
        self.update()
        self.ax1.clear()
        plt.grid(linestyle='--')
        plt.gca().set_aspect('equal', adjustable='box')
        self.draw_bot('b', 1.0)
        self.plot_reference()
        
    def plot_reference(self):
        self.ax1.plot(self.reference_poses[:][0], self.reference_poses[:][1])
        size = 0.01
        for x in range(self.reference_poses.shape[1]):
            pose = self.reference_poses[:, x]
            arrow = mpatches.Arrow(pose[0],
                                   pose[1],
                                   size*np.cos(pose[2]), size*np.sin(pose[2]),
                                   color='r', width=size/2, alpha=0.5)
            self.ax1.add_patch(arrow)
        
    def draw_bot(self, color, alpha, size=0.03):
        arrow = mpatches.Arrow(self.pose[0], self.pose[1], size*np.cos(self.pose[2]), size*np.sin(self.pose[2]),
                               color=color, width=size/2, alpha=alpha)
        self.ax1.add_patch(arrow)
        
    def update(self):
        self.update_state()
        self.update_pose()
        self.get_next_reference_pose()
        # print(list(zip(self.state_labels, self.state)))
        
    def update_state(self):
        self.state += dynamics.motion_model(self.state, self.u, self.dt)
        
    def update_pose(self):
        self.pose[0] = self.state[0]
        self.pose[1] = self.state[1]
        self.pose[2] = self.state[2]
        
    def get_next_reference_pose(self):
        self.reference_poses = np.hstack((np.delete(self.reference_poses, 0, 1), np.ndarray((3,1), buffer=np.array([self.reference_poses[0][-1] + 0.002, 0.75, 0.], ndmin=2))))
        
    def on_key_press(self, event):
        """
        Matplotlib keypress event handler that adjusts the angular velocities of
        the left and right motors of the robot.
        """
        if (event.key == "left"):
            self.u += np.array([-1, 1]) * self.steer_increment
        elif (event.key == "right"):
            self.u += np.array([1, -1]) * self.steer_increment
        elif (event.key == "up"):
            self.u += self.forward_increment
        elif (event.key == "down"):
            self.u -= self.forward_increment
            
    def cost_function(self):
        pass


if __name__ == "__main__":
    mpc = MPC()
    mpc.start_sim()
    