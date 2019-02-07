# ParticleFilter.py -- TODO SUMMARY
import numpy as np

def particle_filter_update(X, u_mu, u_sigma, Z, obs_func):
    """ Update step for a particle filter

        Args:
            X (r x c numpy array): Each column is a particle, each row corresponds to a state var
            u_mu (r x 1 numpy array): Noise mean for motion model
            u_sigma (r x 1 numpy array): Noise stdev 
            Z (s x 1 numpy array): Sensor returns for each sensor
            obs_func ( (s x 1 array), (r x 1 array) -> float): P(Z | x)

        Returns:
            (r x c numpy_array): New particles
    """
    
    n_particles = X.shape[1]
    n_dimensions = X.shape[0]

    # Sample the motion model of existing particles
    X_perturbed = X + np.random.normal(u_mu, u_sigma, size=(n_particles,n_dimensions)).transpose()

    # find P(Z | x) for each perturbed particle
    likelihoods = np.zeros(n_particles)
    for ix in range(n_particles):
        likelihoods[ix] = obs_func(Z, X_perturbed[:,ix])

    # The most likely sample will be resampled at most this much more frequently than the least
    # likely
    max_resample_weight = 500

    X_bel_prob = np.zeros(n_particles)
    # Resample the particles weighted by P(Z | x)
    for m in range(n_particles):
        if m == 1:
            ix = 1
        else:
            ix = m-1

        X_bel_prob[m] = X_bel_prob[ix] + max_resample_weight*likelihoods[m]
        # each "segment" of this has length proportional to the weight

    X_new = resample(X_perturbed, X_bel_prob)

    return X_new

def resample(X, prob):
    """ Resamples the Particles in X based on the probability vector

        Args:
            X (r x c numpy array): Samples from the columns of X (with replacement) based on the 
                probabilities in prob
            prob ([float]): An increasing list of numbers, where (prob[i+1]-prob[i])/prob[-1]
                represents the probability of of x_i

            Returns:
                r x c numpy array: Resampled values of X
    """

    (_,n_samples) = X.shape
    sample_indices = sorted(np.random.uniform(low=0, high=max(prob), size=n_samples))

    X_resamp = np.zeros(X.shape)
    nx = 0
    for ix in range(len(prob)):
        cprob = prob[ix]
        while (nx < len(sample_indices)) and (cprob > sample_indices[nx]):
            X_resamp[:,nx] = X[:,ix]
            nx += 1

    return X_resamp
