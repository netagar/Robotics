from grid import *
from particle import Particle
from utils import *
from setting import *

import numpy as np
from numpy import *
import scipy.stats
import random



def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
    
    for particle in particles:
        (dx, dy, dh) = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        (rx, ry) = rotate_point(dx, dy, particle.h)
        motion_particles.append(Particle(particle.x + rx, particle.y + ry, particle.h + dh))
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
   
    weights = []
    
    for particle in particles:
        particle_markers_list = particle.read_markers(grid)
        prob = sum([marker_probability(measured, observed)
                    for measured in particle_markers_list
                    for observed in measured_marker_list])
        weights.append(prob)

    weigt_sum = sum(weights)
    if weigt_sum > 0:
        weights /= sum(weights)

    
    indexes = systematic_resample(weights)

    # resample according to indexes
    measured_particles = []
    for index in indexes:
        measured_particles.append(particles[index])
    return measured_particles

def marker_probability(measurd, observed):
    (dx, dy, dh) = (measurd[0] - observed[0], measurd[1] - observed[1], measurd[2] - observed[2])
    return scipy.stats.norm(0, MARKER_TRANS_SIGMA).pdf(dx) * scipy.stats.norm(0, MARKER_TRANS_SIGMA).pdf(dy) * scipy.stats.norm(0, MARKER_ROT_SIGMA).pdf(dh)

def systematic_resample(weights):
    N = len(weights)

    positions = (np.arange(N) + random.random()) / N

    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0
    while i < N & j < N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes


