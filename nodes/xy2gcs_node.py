#! /usr/bin/env python

import sys
import rospy
from GCSTools import *
from sensor_msgs.msg import NavSatFix
from sightedturtlesim.msg import PoseXYZ

class XY2GCSNode:
  def __init__(self):
    self.home_lat = 45.504824264187214
    self.home_lon = -73.57661962509155
    
    home_lat = rospy.get_param('~home_lat', self.home_lat)
    home_lon = rospy.get_param('~home_lon', self.home_lon)
    try:
      home_lat_val = float(home_lat)
      if abs(home_lat_val) <= 90.:
        self.home_lat = home_lat_val
      else:
        rospy.logwarn('Specified ~home_lat not within [-90,90] range')
      home_lon_val = float(home_lon)
      if abs(home_lon_val) <= 180.:
        self.home_lon = home_lon_val
      else:
        rospy.logwarn('Specified ~home_lon not within [-180,180] range')
    except BaseException:
      rospy.logwarn('Failed to parse ~home_lat/~home_lon')
    
    self.gcs_pub = rospy.Publisher('/turtle1/pose_gcs', NavSatFix, queue_size=1)
    self.pose_sub = rospy.Subscriber('/turtle1/pose_xyz', PoseXYZ, self.poseCB)
    rospy.loginfo('XY2GCS node initiated with home lat/lon: %.16f, %.16f' % (self.home_lat, self.home_lon))
  
  def poseCB(self, xyz_msg):
    latlon = Meter2GCS(self.home_lat, self.home_lon, xyz_msg.x, xyz_msg.y)
    gcs_msg = NavSatFix()
    gcs_msg.header.frame_id = xyz_msg.header.frame_id
    gcs_msg.header.stamp = xyz_msg.header.stamp
    gcs_msg.latitude = latlon[0]
    gcs_msg.longitude = latlon[1]
    gcs_msg.altitude = xyz_msg.z
    gcs_msg.position_covariance = [-1]*9
    gcs_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    self.gcs_pub.publish(gcs_msg)
    
  def spin(self):
    rospy.spin()
    

if __name__ == '__main__':
  rospy.init_node('xy2gcs_node')
  node = XY2GCSNode()
  node.spin()
