#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ParticleFilter import particle_filter_update

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
                self.zpoints[iy][ix] = self.gaussian_2d(self.mu, self.sigma, x, y)

        self.n_parts = 1000
        self.real_obj = np.array([-4,-4])
        self.particles = np.random.uniform(low=-5,high=5, size=(self.n_parts, 2))
        self.u_mu = np.array([.1,.1])
        self.u_sigma = np.array([.05,.05])

    def gaussian_2d(self, mu, sigma, x, y):
        X = np.array([x,y])
        return np.exp(-1/2 * np.dot((X - mu).transpose(),np.dot(np.linalg.inv(sigma),(X - mu))))/(np.sqrt((2*np.pi)**2 * np.linalg.det(sigma)))

    def obs_func(self, Z, x):
        """ P(Z | x) """
        z_exp = self.gaussian_2d(self.mu, self.sigma, x[0], x[1])
        return np.exp(-(100*(z_exp - Z))**2)


    def animate_plot(self, i):

        self.real_obj = self.real_obj + np.random.normal(self.u_mu, self.u_sigma)
        #Z = gaussian_2d(self.mu, self.sigma, self.real_obj[0], self.real_obj[1]) + np.random.normal(0, .01)
        Z = self.gaussian_2d(self.mu, self.sigma, self.real_obj[0], self.real_obj[1])

        self.particles = particle_filter_update(self.particles, self.u_mu, self.u_sigma, Z, self.obs_func)
        ax1.clear()
        ax1.imshow(self.zpoints,extent=[-5,5,-5,5])
        ax1.scatter(self.particles[:,0], self.particles[:,1], color='r', alpha=.2)
        ax1.scatter(self.real_obj[0], self.real_obj[1], color='b')


if __name__ == "__main__":
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    pf = SimpleParticleFilterTest()
    num_iterations = 80
    animation = animation.FuncAnimation(fig, pf.animate_plot, frames=num_iterations, repeat=False, interval=10)
    plt.show()
