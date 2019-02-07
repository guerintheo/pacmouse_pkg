#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from ParticleFilter import particle_filter_update

mu = np.array([0,0])
sigma = np.array([[5,0],[0,5]])

def gaussian_2d(mu, sigma, x, y):
    X = np.array([x,y])
    return np.exp(-1/2 * (X - mu).transpose() @ np.linalg.inv(sigma) @ (X - mu))/(np.sqrt((2*np.pi)**2 * np.linalg.det(sigma)))

def obs_func(Z, x):
    """ P(Z | x) """
    z_exp = gaussian_2d(mu, sigma, x[0], x[1])
    return np.exp(-(100*(z_exp - Z))**2) 

xpoints = np.linspace(-5,5,100)
ypoints = np.linspace(-5,5,100)
zpoints = np.zeros((len(ypoints),len(xpoints)))
for ix in range(len(xpoints)):
    for iy in range(len(ypoints)):
        x = xpoints[ix]
        y = ypoints[iy]
        zpoints[iy][ix] = gaussian_2d(mu,sigma, x, y)


plt.imshow(zpoints)
plt.show()

n_parts = 1000
real_obj = np.array([-4,-4])
particles = np.random.uniform(low=-5,high=5, size=(2,n_parts))
u_mu = np.array([.1,.1])
u_sigma = np.array([.1,.1])

for step in range(100):
    real_obj = real_obj + np.random.normal(u_mu, u_sigma)
    #Z = gaussian_2d(mu, sigma, real_obj[0], real_obj[1]) + np.random.normal(0, .01)
    Z = gaussian_2d(mu, sigma, real_obj[0], real_obj[1])

    particles = particle_filter_update(particles, u_mu, u_sigma, Z, obs_func)
    plt.imshow(zpoints,extent=[-5,5,-5,5])
    plt.scatter(particles[0,:],particles[1,:],color='r',alpha=.2)
    plt.scatter(real_obj[0], real_obj[1], color='b')
    plt.show()
