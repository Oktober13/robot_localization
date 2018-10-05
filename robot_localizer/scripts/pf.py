#!/usr/bin/env python

""" This is the starter code for the robot localization project """

import rospy
import math
import atexit

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from helper_functions import TFHelper
from occupancy_field import OccupancyField
from ros_boss import RosBoss


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.ros_boss = RosBoss()

        self.current_particles = {}
        self.max_particle_number = 100
        self.map_width = self.occupancy_field.map.info.width
        self.map_height, self.occupancy_field.map.info.height

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # TODO this should be deleted before posting
        self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # initialize your particle filter based on the xy_theta tuple

    def generate_initial_particles(self):
        """ Generate a number of initial particles and add them to the dict. """

        num_created_particles = self.max_particle_number - len(self.current_particles)

        for num in range(num_created_particles):
            # Generate random x,y coordinate for particle, and add as key-value pair to dict of particles
            self.current_particles[random.uniform(0.0, self.map_width)] = random.uniform(0.0, self.map_height)
        return
        
    def sample_best_particles(self):
        """ Iterates through list of particles and recycles the ones below the threshold. """
        pass

    def thresholdWeight(self):
        """ Picks the threshold value below which particles will be recycled. """
        pass

    def generate_probability_dist(self):
        """ Generates more particles around the existing most likely particles. """
        pass

    def calc_new_weights(self):
        """ Iterates through list of particles and calculates new weights. """
        pass

    def estimate_future_particles(self):
        """ Estimate future particle locations given noise and current speed. """

        for key in self.current_particles:
            x,y = key, self.current_particles[key]

            #Some stuff estimating distance using velocity twist message and timesince.
        pass

    def normalize_weights(self):
        """ Normalizes all weights to be between 0 and 1. """
        pass

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
