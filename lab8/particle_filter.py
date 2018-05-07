from grid import *
from particle import Particle
from utils import *
from setting import *
from math import sin, cos, radians
import random
from scipy import stats
import numpy as np

MARKER_ANGLE_SIGMA = 10
MARKER_DIST_SIGMA = 0.5

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
        # add noise to each particle's odom then move it accordingly
        new_particle = Particle(particle.x, particle.y, particle.h)
        new_odom = add_odometry_noise(odom, heading_sigma=ODOM_HEAD_SIGMA, trans_sigma=ODOM_TRANS_SIGMA)
        dx, dy = rotate_point(new_odom[0], new_odom[1], particle.h)
        new_particle.x += dx
        new_particle.y += dy
        new_particle.h += new_odom[2]
        
        motion_particles.append(new_particle)
    
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
    measured_particles = []
    particles_list = []
    particles_weight = []
    marker_list= []
    match_list = []
    weight_sum = 0.0
    num_robot_markers = len(measured_marker_list)
    resampling_count = 0
    
    for particle in particles:
        # throw away particles that went out of bound
        if not grid.is_in(particle.x, particle.y):
            continue
            
        # the list of marker seen by a particle
        particle_marker_list = particle.read_markers(grid)
        num_particle_markers = len(particle_marker_list)
        weight = 0.0
        #marker noise add_marker_measurement_noise(m, trans_sigma=MARKER_TRANS_SIGMA, rot_sigma=MARKER_ROT_SIGMA)
        if num_robot_markers == 0 and num_particle_markers == 0:
            # no markers seen
            # minimum weight. it is not informative
            weight = stats.norm.pdf(2*MARKER_ANGLE_SIGMA, loc=0.0, scale=MARKER_ANGLE_SIGMA)
        elif num_robot_markers > 0 and num_particle_markers > 0:
            weight = 0.0
            # both robot and particle see some number of markers
            for robot_marker in measured_marker_list:
                for particle_maker in particle_marker_list:
                    #particle_maker = add_marker_measurement_noise(particle_maker, trans_sigma=MARKER_TRANS_SIGMA, rot_sigma=MARKER_ROT_SIGMA)
                    # difference between robot's angle to marker and particle's angle to marker
                    angle_diff = abs(robot_marker[2] - particle_maker[2])
                    # distance between robot and marker
                    robot_dist = math.sqrt(robot_marker[0]**2 + robot_marker[1]**2)
                    # distance between particle and marker
                    particle_dist = math.sqrt(particle_maker[0]**2 + particle_maker[1]**2)
                    # difference between robot's distance to marker and particle's angle to distance
                    distance_diff = abs(robot_dist - particle_dist)

                    angle_weight = stats.norm.pdf(angle_diff, loc=0.0, scale=MARKER_ANGLE_SIGMA)
                    distance_weight = stats.norm.pdf(distance_diff, loc=0.0, scale=MARKER_DIST_SIGMA)
                    weight += (angle_weight * distance_weight)
            weight = weight / (num_robot_markers * num_particle_markers)
        else:
            # only robot or particle see any marker
            # unlikely match
            weight = stats.norm.pdf(2*MARKER_ANGLE_SIGMA, loc=0.0, scale=MARKER_ANGLE_SIGMA)
        
        particles_list.append(particle)
        # Multiply by a billion because the weight are very small numbers most of the time
        # and are rounded to zero too readily
        particles_weight.append(weight*1000000000) 
        marker_list.append(particle_marker_list)
        match_list.append((num_particle_markers, num_robot_markers))
        weight_sum += weight*1000000000
        
    for i in range(len(particles_list)):
        normalized_weight = particles_weight[i] / weight_sum
        resampling_count = np.around(PARTICLE_COUNT * normalized_weight)
                
        # resampling
        for _ in range(math.floor(resampling_count)):
            measured_particles.append(Particle(particles_list[i].x, particles_list[i].y, particles_list[i].h))
    measured_particles.extend(Particle.create_random(100, grid))
    return measured_particles


