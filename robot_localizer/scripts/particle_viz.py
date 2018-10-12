#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

import rospy

class ParticlesMarker(object):
    """ 
    Particles created by the particle filter.
    """
    def __init__(self):
        solid_red = ColorRGBA()
        solid_red.r = 1.0 # Opaque red
        solid_red.g = 0.0
        solid_red.b = 0.0
        solid_red.a = 1.0

        self.seq = 0
        self.scale = Vector3( 0.1, 0.1, 0.1 )
        self.color = solid_red

        self.pub = rospy.Publisher('/marker', Marker, queue_size = 10)
        self.r = rospy.Rate(0.1) # 10 Hz, or 10 cycles per second.
    
    def updateParticles(self, particles):
        points = []
        colors = []
        self.seq += 1
        header = Header(seq = self.seq, stamp = rospy.get_rostime(), frame_id='map')

        # Create array of points for each particle coordinate
        for particle in particles.keys():
            points.append(Point(particle.x, particle.y, 0.0))
            colors.append(self.color)

        markers = Marker( 
            header = header,
            ns = "markers", 
            id = 0, 
            type = 8,
            action = 0,
            pose = None,
            scale = self.scale,
            color = self.color,
            lifetime = None,
            frame_locked = False,
            points = points,
            colors = colors,
            text = None,
            mesh_resource = None,
            mesh_use_embedded_materials = None
        )

        self.pub.publish( markers )