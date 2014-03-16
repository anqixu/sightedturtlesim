#!/usr/bin/env python
import roslib
roslib.load_manifest('sightedturtlesim')
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sightedturtlesim.msg import ImageWithPoseXYZ
from sightedturtlesim.srv import QueryGeolocatedImage


class robbie:
  def __init__(self):
    cv.NamedWindow("Source", 1)
    cv.NamedWindow("Replay", 2)
    self.bridge = CvBridge()
    rospy.init_node('listener', anonymous=True)
    rospy.wait_for_service('/turtle1/query_geolocated_image')
    self.query = rospy.ServiceProxy('/turtle1/query_geolocated_image', QueryGeolocatedImage)
    self.sub = rospy.Subscriber("/turtle1/image_xyz", ImageWithPoseXYZ, self.callback)
    
  def callback(self, msg):
    try:
      queryImg = self.query(msg.pose)
    except rospy.ServiceException, e:
      print "Service call failed: %s" % e
      return

    match = True
    if msg.img.height != queryImg.img.height:
      print 'ERROR: Height mismatch: %d / %d' % (msg.img.height, queryImg.img.height)
      return
    elif msg.img.width != queryImg.img.width:
      print 'ERROR: Width mismatch: %d / %d' % (msg.img.width, queryImg.img.width)
      return
    elif msg.img.encoding != queryImg.img.encoding:
      print 'ERROR: encoding mismatch: %s != %s' % (msg.img.encoding, queryImg.img.encoding)
      return
    elif msg.img.is_bigendian != queryImg.img.is_bigendian:
      print 'ERROR: is_bigendian mismatch'
      return
    elif msg.img.step != queryImg.img.step:
      print 'ERROR: step mismatch: %d / %d' % (msg.img.step, queryImg.img.step)
      return
    else:
      size = msg.img.step * msg.img.height
      for i in xrange(size):
        if msg.img.data[i] != queryImg.img.data[i]:
          match = False
          print 'ERROR: value mismatch at index %d (%d)' % (i, msg.img.header.seq)
          break
    if match:
      print 'SUCCESS: img match (%d)' % msg.img.header.seq

    try:
      sourceImg = self.bridge.imgmsg_to_cv(msg.img, desired_encoding="bgr8")
      replayImg = self.bridge.imgmsg_to_cv(queryImg.img, desired_encoding="bgr8")
    except CvBridgeError, e:
      print "CvBridge error: %s" % e
      return

    cv.ShowImage("Source", sourceImg)
    cv.ShowImage("Replay", replayImg)
    cv.WaitKey(3)


if __name__ == '__main__':
  r = robbie()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

