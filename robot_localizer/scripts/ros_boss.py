#!/usr/bin/env python
""" Contains all callbacks for the particle filter project. """

import rospy
import math
import atexit
import statistics

from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class RosBoss(object):
	""" Useful callbacks for the neato. """
	def __init__(self):
		scan = rospy.Subscriber("/stable_scan", LaserScan, self.laserCallback)
		odom = rospy.Subscriber("/odom", Odometry, self.odomCallback)
		initialpose = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.initialPoseCallback)
		# ^ Should be /initialpose, but that topic's not showing up for me?

		self.stamped_pose = None
		self.linear_vel = None
		self.ranges = None
		self.minDistance = 1000000.0
		self.minAngle = 0
		return

	def laserCallback(self, msg):
		self.ranges = msg.ranges
		minDistance = sum(msg.ranges) # Some large value

		for angle in range(len(msg.ranges)):
			distance = msg.ranges[angle]
			# Person will be closer to the Neato than most things.
			if ((distance < minDistance) and (distance != 0.0)):
				self.minDistance = distance
				self.minAngle = angle
		return

	def odomCallback(self, msg):
		if self.poseGiven(): # Ensures PF will not run without a pose value from the user.
			self.stamped_pose = msg.pose # this is a PoseStamped
			self.linear_vel = msg.twist.twist.linear
		return

	def initialPoseCallback(self, msg): # User set the initial pose.
		self.stamped_pose = PoseStamped(
			header=msg.header, 
			pose=msg.pose)
		return

	def poseGiven(self):
		if (self.stamped_pose is not None):
			return True
		else:
			return False