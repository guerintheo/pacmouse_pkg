# ParticleFilter.py -- TODO SUMMARY
import numpy as np

def particle_filter_update(X, u_mu, u_sigma, Z, obs_func):
    """ Update step for a particle filter

        Args:
            X (c x r numpy array): Each row is a particle, each column corresponds to a state var
            u_mu (r x 1 numpy array): Noise mean for motion model
            u_sigma (r x 1 numpy array): Noise stdev
            Z (s x 1 numpy array): Sensor returns for each sensor
            obs_func ( (s x 1 array), (r x 1 array) -> float): P(Z | x)

        Returns:
            (r x c numpy_array): New particles
    """
    # Sample the motion model of existing particles
    X_perturbed = X + np.random.normal(u_mu, u_sigma, size=X.shape)
    # find P(Z | x) for each perturbed particle
    likelihoods = np.array([obs_func(Z, x) for x in X_perturbed])

    # make likelihoods into a pmf
    likelihoods += np.min(likelihoods)
    likelihoods /= np.sum(likelihoods)

    # resample the particles with replacement
    return resample(X_perturbed, likelihoods)

def resample(X, pmf):
    """ Given c particles and a c-element pmf, choose c new particles from the input particles
    with replacement, according to the specified pmf

    Args:
        X (c x r numpy array): Each row is a particle, each column corresponds to a state var
        pmf (c x 1 numpy array): the likelihood of each particle given the sensor data

    Returns:
        TYPE: Description
    """
    new_ix = np.random.choice(len(pmf), size=X.shape[0], replace=True, p=pmf)
    return X[new_ix, :]


# NOTE(izzy): the update function should have the same functionality as the one above
class ParticleFilter:
    def __init__(self, particles):
        self.particles = particles
        self.n, self.d = self.particles.shape
        self.likelihoods = np.ones(self.n)/self.n
        self.pmf = self.likelihoods

    def resample(self):
        new_ix = np.random.choice(self.n, size=self.n, replace=True, p=self.pmf)
        self.particles = self.particles[new_ix, :]

    def perturb(self, u_mu, u_sigma):
        self.particles += np.random.normal(u_mu, u_sigma, size=[self.n, self.d])

    def calc_likelihoods(self, Z, obs_func):
        self.likelihoods = np.array([obs_func(Z, x) for x in self.particles])
        self.pmf = self.likelihoods + np.min(self.likelihoods)
        self.pmf /= np.sum(self.pmf)

    def update(self, u_mu, u_sigma, Z, obs_func):
        self.perturb(u_mu, u_sigma)
        self.calc_likelihoods(Z, obs_func)
        self.resample()
