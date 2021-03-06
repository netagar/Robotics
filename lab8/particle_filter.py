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
    cum_prob = 0
    for particle in particles:
        particle_markers_list = particle.read_markers(grid)
        prob = sum([marker_probability(measured, observed)
                    for measured in particle_markers_list
                    for observed in measured_marker_list])
        cum_prob += prob
        weights.append(cum_prob)

    resample = int(len(particles) * 0.95)
    new_sample = len(particles) - int(len(particles) * 0.95)

    if cum_prob == 0:
        sampled = random.choices(particles, k=resample)
    else:
        sampled = random.choices(particles, cum_weights=weights, k=resample)

    sampled.extend(Particle.create_random(new_sample, grid))
    return sampled

def pdf(mean, sigma, x):
    return math.exp(-(x - mean) ** 2 / (2 * sigma ** 2)) / math.sqrt(2 * math.pi * sigma ** 2)


def marker_probability(measurd, observed):
    (dx, dy, dh) = (measurd[0] - observed[0], measurd[1] - observed[1], measurd[2] - observed[2])
    return pdf(0, MARKER_TRANS_SIGMA, dx) * pdf(0, MARKER_TRANS_SIGMA, dy) * pdf(0, MARKER_ROT_SIGMA, dh)



