#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from ParticleFilter import particle_filter_update

class StateEstimatorParticleFilter(object):
    
    def __init__(self):
        self.n_dims = 6
        self.real_obj_state = np.array([-4.0, -4.0, np.pi/4, 0.0, 0.0, 0.0])
        self.initialize_background_gaussian()
        
        self.n_parts = 2000
        self.particles = np.random.uniform(low=-5,high=5, size=(self.n_parts, self.n_dims))
        # Noise due to motion
        self.u_sigma = np.array([.008, .008, .008, .008, .008, .008])
        
        # Some geometric parameters for kinematics. These are just test values
        self.wheel_radius = 0.03 # 0.03 meters, 3 cm
        self.wheel_dist_x = 0.015 # 0.015 meters, 1.5 cm
        self.wheel_dist_y = 0.04 # 0.04 meters, 4 cm
        self.gear_ratio = 3*29.86
        
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
                self.zpoints[iy][ix] = self.gaussian_2d(self.mu, self.sigma, x, y)
    
    def gaussian_2d(self, mu, sigma, x, y):
        X = np.array([x,y])
        return np.exp(-1/2 * np.dot((X - mu).transpose(),np.dot(np.linalg.inv(sigma),(X - mu))))/\
               (np.sqrt((2*np.pi)**2 * np.linalg.det(sigma)))

    def motion_model_mean(self, x, u, dt):
        """
        Given the previous state (x) and control input (u), calculate the
        predicted change in state at the next time step.
        
        x: previous state
        u: control input; NumPy array containing [angular_velocity_left_motor,
           angular_velocity_right_motor]
        dt: time step
        """
        # From robot kinematics. See LaTeX document for this math
        v = self.wheel_radius/2.0*((u[0]+u[1])/self.gear_ratio)
        psi_dot = (self.wheel_radius*self.wheel_dist_y)/(2*(self.wheel_dist_x**2 + self.wheel_dist_y**2))*((u[1]-u[0])/self.gear_ratio)
        # TODO: Handle the psi_dot = 0 case perhaps a bit better, to avoid division by zero
        if psi_dot == 0:
            psi_dot = 0.00001
        
        psi = x[2]
        a = x[4]
        
        dpsi = psi_dot*dt
        dv = a*dt
        
        dx = 1.0/(psi_dot**2.0)*((v+dv)*psi_dot*np.sin(psi + dpsi) - v*psi_dot*np.sin(psi) + a*np.cos(psi + dpsi) - a*np.cos(psi))
        dy = 1.0/(psi_dot**2.0)*((-v-dv)*psi_dot*np.cos(psi + dpsi) + v*psi_dot*np.cos(psi) + a*np.sin(psi + dpsi) - a*np.sin(psi))
        
        change_in_state = np.array([dx, dy, dpsi, dv, 0.0, 0.0])
        return change_in_state

    def obs_func(self, Z, x):
        """ P(Z | x) """
        z_exp = self.gaussian_2d(self.mu, self.sigma, x[0], x[1])
        return np.exp(-(100*(z_exp - Z))**2)

    def animate_plot(self, i, pf):
        mean_change = self.motion_model_mean(self.real_obj_state, np.array([self.omega_l, self.omega_r]), 0.2)
        # Add motion and noise to real robot
        self.real_obj_state += np.random.normal(mean_change, self.u_sigma)
        
        #Z = gaussian_2d(self.mu, self.sigma, self.real_obj_state[0], self.real_obj_state[1]) + np.random.normal(0, .01)
        Z = self.gaussian_2d(self.mu, self.sigma, self.real_obj_state[0], self.real_obj_state[1])

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
    fig = plt.figure(figsize=(14, 7))
    ax1 = fig.add_subplot(1,1,1)
    pf = StateEstimatorParticleFilter()
    animation = animation.FuncAnimation(fig, pf.animate_plot, fargs=(pf,), interval=10)
    cid = fig.canvas.mpl_connect('key_press_event', pf.on_key_press)
    plt.show()
