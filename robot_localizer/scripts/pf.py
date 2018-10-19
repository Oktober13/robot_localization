#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
from geometry_msgs.msg import Point, PoseArray, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from helper_functions import TFHelper # ROS tf package transform helper functions
from occupancy_field import OccupancyField
from particle_viz import ParticlesMarker
from ros_boss import RosBoss

import rospy
import math
import numpy as np
import time
# import atexit

class Particle(object):
    """ Object for storing particle data. """
    def __init__(self, x, y, theta, distance = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.distance = distance

class ParticleFilter(object):
    """ 
    The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.tfh = TFHelper()
        self.ros_boss = RosBoss()

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)
        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("/odom",
                         Odometry,
                         self.update_initial_pose)

        self.current_particles = {}
        self.max_particle_number = 100
        self.map_width = self.occupancy_field.map.info.width
        self.map_height = self.occupancy_field.map.info.height
        self.map_origin = (self.occupancy_field.map.info.origin.position.x, self.occupancy_field.map.info.origin.position.y)
        self.map_resolution = self.occupancy_field.map.info.resolution
        self.particle_viz = ParticlesMarker()

        self.last_time = time.time()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """

        self.tfh.fix_map_to_odom_transform(msg.pose.pose, msg.header.stamp) # Update transform between map and odom.

        if self.ros_boss.poseGiven():
            xy_theta = self.tfh.convert_pose_to_xy_and_theta(self.ros_boss.stamped_pose) # Get position and orientation from pose

            if len(self.current_particles) == 0: #Initial generation of particles
                self.generate_initial_particles(xy_theta)
            else:
                # self.current_particles = {(Particle : probability}
                threshold = self.thresholdWeight() # Calculate threshold weight
                self.sample_best_particles(threshold) # self.current_particles = selected best particles.
                self.generate_probability_dist() # "Refills" any missing particles by placing them around the most likely current particles.
                for particle in self.current_particles.keys():
                    self.calc_new_weights(particle) # Calculates new probabilities for all particles (Evaluation)
                    timesince = time.time() - self.last_time
                    self.estimate_future_particles(particle, timesince) # Move particles forward in time
                    self.last_time = time.time()
                    self.normalize_weights(particle, max(self.current_particles.values()))

                # Update RViz map and particles
                self.particle_viz.updateParticles(self.current_particles,
                    self.map_origin,
                    self.map_resolution) # Update rviz visualization

    def generate_initial_particles(self, xy_theta):
        """ Generate a number of initial particles and add them to the dict. x & y are in meters."""
        raw_x, raw_y, theta = xy_theta
        timenow = rospy.get_rostime()

        # self.transform_helper.tf_listener.transformPoint("odom", Point, base_point);
        # + self.transform_helper.translation.y
        for num in range(self.max_particle_number):
            # Generate random x,y, theta coordinate for particle, and add as key-value pair to dict of particles
            [x,y] = np.random.multivariate_normal([raw_x, raw_y], [[0.0, 0.0],[0.0, 0.0]])
            theta = np.random.uniform(0.0, 2 * math.pi)

            # pose = self.tfh.transform(Pose(Point(raw_x, raw_y, 0.0), self.tfh.euler_to_quat(raw_theta)))
            # x, y, theta = self.tfh.convert_pose_to_xy_and_theta(Pose(pose.pose.position, pose.pose.orientation))
            self.current_particles[Particle(x,y,theta)] = (1.0 / self.max_particle_number)

        self.particle_viz.bestParticle = (list(self.current_particles.items()))[0]

        return
        
    def sample_best_particles(self, threshold):
        """ Iterates through list of particles and recycles the ones below the threshold. """

        # Two step process to avoid changing the dict while iterating through
        best_particles = {particle:probability for particle, probability in self.current_particles.iteritems() if self.current_particles[particle] <= threshold}
        self.current_particles = best_particles

        if len(self.current_particles) == 0:
            return None
        return

    def thresholdWeight(self):
        """ Picks the threshold value below which particles will be recycled. """
        return sum(self.current_particles.values()) / len(self.current_particles.values()) # Average value

    def generate_probability_dist(self):
        """ Generates more particles around the existing most likely particles. """
        if len(self.current_particles.values()) < self.max_particle_number:
            num_created_particles = self.max_particle_number - len(self.current_particles.values())
            # Iterate through sorted list of particles to get list of best options
            topProbabilities = dict(sorted(
                self.current_particles.iteritems(), 
                key=lambda (particle,probability): (particle,probability), 
                reverse = True)[:(len(self.current_particles) % num_created_particles)])

            if len(topProbabilities) > 0:
                # Get best particle (first in sorted dict)
                best_particle, probability = (list(topProbabilities.items()))[0]
                # Set value in particle_viz
                self.particle_viz.bestParticlePose = Pose(Point(best_particle.x, best_particle.y, 0.0), self.tfh.euler_to_quat(best_particle.theta))

            # Create new particles.
            preload = math.floor(len(self.current_particles) / num_created_particles) 
            new_particles = {}
            index = 0

            for particle in self.current_particles.keys():
                # if num desired particles is larger than current list of particles, we must create triplicates, etc.
                for num in range(int(preload)):
                    new_particles[Particle(particle.x, particle.y, particle.theta, particle.distance)] = self.current_particles[particle]
                # These are the ones that get an extra first, if the number of new particles is not evenly divisible.
                if (index <= (len(self.current_particles) % num_created_particles)): 
                    new_particles[Particle(particle.x, particle.y, particle.theta, particle.distance)] = self.current_particles[particle]
                index += 1

            # Add new dict entries to self.current_particles
            self.current_particles.update(new_particles)
        return

    def calc_new_weights(self, particle):
        """ Iterates through list of particles and calculates new weights. """
        angle_probability = self.tfh.angle_diff(self.ros_boss.minAngle, particle.theta) / (2 * math.pi)
        distance_probability = abs(self.ros_boss.minDistance - particle.distance) / self.ros_boss.minDistance # Shortest laser scan - shortest laser scan for current particle.
        self.current_particles[particle] = angle_probability + distance_probability
        return

    def estimate_future_particles(self, particle, timesince):
        """ Estimate future particle locations given noise and current speed. Particles are in the map frame. """

        particle.distance = math.sqrt((self.ros_boss.linear_vel.x * timesince)**2 + (self.ros_boss.linear_vel.y * timesince)**2) + np.random.uniform(-0.1, 0.1)
        particle.x = particle.x + self.ros_boss.linear_vel.x * timesince + np.random.uniform(-0.1,0.1) # DeltaX = Velx * time + noise
        particle.y = particle.y + self.ros_boss.linear_vel.y * timesince + np.random.uniform(-0.1,0.1)
        return

    def normalize_weights(self, particle, max_val):
        """ Normalizes all weights to be between 0 and 1. """

        self.current_particles[particle] = self.current_particles[particle] / max_val
        return

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.tfh.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
