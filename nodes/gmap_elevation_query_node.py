#! /usr/bin/env python

import sys
import rospy
import googlemaps # sudo pip install -U googlemaps
from GCSTools import *
from sightedturtlesim.msg import ImageWithPoseXYZ

import requests.packages.urllib3
requests.packages.urllib3.disable_warnings()

# TODO: parse top-left lat, lon
topleft_latlon = (45.518436303950104, -73.62297892570496)

class GoogleMapElevationQueryNode:
  def __init__(self, server_key):
    self.gmaps = googlemaps.Client(key=server_key)
    self.pose_sub = rospy.Subscriber('/turtle1/image_xyz', ImageWithPoseXYZ, self.imageWithPoseCB)
  
  def imageWithPoseCB(self, msg):
    curr_latlon = Meter2GCS(topleft_latlon[0], topleft_latlon[1], msg.pose.x, msg.pose.y)
    curr_altitude = self.gmaps.elevation(curr_latlon)[0]['elevation']

    rospy.loginfo('x=%.2f, y=%.2f; lat=%.8f, lon=%.8f\n  alt: %f' % (msg.pose.x, msg.pose.y, curr_latlon[0], curr_latlon[1], curr_altitude))
    
  def spin(self):
    rospy.spin()
    

if __name__ == '__main__':
  server_key = ''
  rospy.init_node('gmap_elevation_query_node')
  if rospy.has_param('~server_key'):
    server_key = rospy.get_param('~server_key')
  
  if not(len(server_key) == 39):
    sys.exit('Need to set ~server_key (e.g. rosrun ... _server_key:=''...'') to a valid Google API Server Key with access to Google Maps Elevations API')
  else:
    node = GoogleMapElevationQueryNode(server_key)
    node.spin()
