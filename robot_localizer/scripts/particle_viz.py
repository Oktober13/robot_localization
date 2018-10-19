#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

import rospy

class ParticlesMarker(object):
    """ 
    Particles created by the particle filter.
    """
    def __init__(self):
        solid_blue = ColorRGBA()
        solid_blue.r = 0.0 # Opaque blue
        solid_blue.g = 0.0
        solid_blue.b = 1.0
        solid_blue.a = 1.0

        self.seq = 0
        self.color = solid_blue
        self.bestParticlePose = None

        self.pub = rospy.Publisher('/marker', MarkerArray, queue_size = 10)
        self.r = rospy.Rate(0.1) # 10 Hz, or 10 cycles per second.
    
    def updateParticles(self, particles, origin_coords, scale):
        points = []
        colors = []
        self.seq += 1
        header = Header(seq = self.seq, stamp = rospy.get_rostime(), frame_id='map')

        # Create array of points for each particle coordinate
        for particle in particles.keys():
            x,y = self.convertMeterToPixel((particle.x, particle.y), origin_coords, scale) # Convert meters to pixels
            points.append(Point(x,y, 0.0))
            colors.append(self.color)

        # Create Marker message for particle cloud and best indication arrow.
        markers = Marker( 
            header = header,
            ns = "markers", 
            id = 0, 
            type = 2,
            action = 0,
            pose = None,
            scale = Vector3( 0.1, 0.1, 0.1 ),
            color = self.color,
            lifetime = None,
            frame_locked = False,
            points = points,
            colors = colors,
            text = None,
            mesh_resource = None,
            mesh_use_embedded_materials = None
        )

        if self.bestParticlePose is not None:
            # Convert particlePose Point position to pixels
            self.bestParticlePose.position.x, self.bestParticlePose.position.y = self.convertMeterToPixel(
                (self.bestParticlePose.position.x, self.bestParticlePose.position.y), origin_coords, scale)

            bestArrow = Marker( 
                header = header,
                ns = "bestArrow", 
                id = 0, 
                type = 0,
                action = 0,
                pose = self.bestParticlePose,
                scale = Vector3(1.0, 0.05, 0.05),
                color = self.color,
                lifetime = None,
                frame_locked = False,
                points = None,
                colors = None,
                text = None,
                mesh_resource = None,
                mesh_use_embedded_materials = None
            )
            
            self.pub.publish( MarkerArray([markers, bestArrow]) )
        else:
            self.pub.publish( MarkerArray([markers]) 
)
    def convertMeterToPixel(self, passed_xy, passed_origin, passed_scale):
        x,y = passed_xy
        x_origin, y_origin = passed_origin

        pixel_x = ((x - x_origin) * passed_scale)
        pixel_y = ((y - y_origin) * passed_scale)

        return pixel_x, pixel_y