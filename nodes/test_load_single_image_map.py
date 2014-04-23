#!/usr/bin/env python
import roslib; roslib.load_manifest('sightedturtlesim')
import sys
import rospy
from sightedturtlesim.srv import *


def loadSingleImageMap(path, ppm):
  rospy.wait_for_service('/sightedturtlesim/load_single_image_map')
  try:
    client = rospy.ServiceProxy('/sightedturtlesim/load_single_image_map', LoadSingleImageMap)
    resp = client(path, ppm)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


if __name__ == "__main__":
  path = "/home/thalassa/anqixu/Dropbox/MAPS/barbados_inland.jpg";
  ppm = 10.0;
  
  loadSingleImageMap(path, ppm)
