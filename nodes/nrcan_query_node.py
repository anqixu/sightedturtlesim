#! /usr/bin/env python

import rospy
import urllib
from GCSTools import *
from sightedturtlesim.msg import ImageWithPoseXYZ

# http://www.nrcan.gc.ca/earth-sciences/geography/topographic-information/free-data-geogratis/geogratis-web-services/api/17328
def query_altitude(lat, lon):
  url = 'http://geogratis.gc.ca/services/elevation/cdem/altitude?lat=%f&lon=%f' % (lat, lon)
  page = urllib.urlopen(url)
  altitude = float('nan')
  for line in page.readlines():
    start_idx = line.find('"altitude": ')
    if start_idx >= 0:
      end_idx = line.rfind(',')
      if end_idx >= 0:
        altitude = float(line[start_idx+12:end_idx])
        break
  return altitude
  
# TODO: parse top-left lat, lon
topleft_latlon = (45.518436303950104, -73.62297892570496)

class NRCANQueryNode:
  def __init__(self):
    self.pose_sub = rospy.Subscriber('/turtle1/image_xyz', ImageWithPoseXYZ, self.imageWithPoseCB)
  
  def imageWithPoseCB(self, msg):
    curr_latlon = Meter2GCS(topleft_latlon[0], topleft_latlon[1], msg.pose.x, msg.pose.y)
    curr_altitude = query_altitude(curr_latlon[0], curr_latlon[1])

    rospy.loginfo('x=%.2f, y=%.2f; lat=%.8f, lon=%.8f\n  alt: %f' % (msg.pose.x, msg.pose.y, curr_latlon[0], curr_latlon[1], curr_altitude))
    
  def spin(self):
    rospy.spin()
    

if __name__ == '__main__':
  rospy.init_node('nrcan_query_node')
  node = NRCANQueryNode()
  node.spin()
