#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ParticleFilter import particle_filter_update

class ParticleFilterTests(object):

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
        return np.exp(-1/2 * (X - mu).transpose() @ np.linalg.inv(sigma) @ (X - mu))/(np.sqrt((2*np.pi)**2 * np.linalg.det(sigma)))

    def obs_func(self, Z, x):
        """ P(Z | x) """
        z_exp = self.gaussian_2d(self.mu, self.sigma, x[0], x[1])
        return np.exp(-(100*(z_exp - Z))**2)

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate_plot(i, pf):

    pf.real_obj = pf.real_obj + np.random.normal(pf.u_mu, pf.u_sigma)
    #Z = gaussian_2d(pf.mu, pf.sigma, pf.real_obj[0], pf.real_obj[1]) + np.random.normal(0, .01)
    Z = pf.gaussian_2d(pf.mu, pf.sigma, pf.real_obj[0], pf.real_obj[1])

    pf.particles = particle_filter_update(pf.particles, pf.u_mu, pf.u_sigma, Z, pf.obs_func)
    ax1.clear()
    ax1.imshow(pf.zpoints,extent=[-5,5,-5,5])
    ax1.scatter(pf.particles[:,0], pf.particles[:,1], color='r', alpha=.2)
    ax1.scatter(pf.real_obj[0], pf.real_obj[1], color='b')

if __name__ == "__main__":
    pf = ParticleFilterTests()
    num_iterations = 80
    animation = animation.FuncAnimation(fig, animate_plot, frames=num_iterations, repeat=False, fargs=(pf,), interval=10)
    plt.show()
