#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from sightedturtlesim.msg import VelocityXYZ, DriveXYZ, TurtleParams
from sightedturtlesim.srv import SetTurtleParams


class Teleop:
  def __init__(self):
    self.params = TurtleParams()
    
    rospy.init_node('teleop_sighted_turtle_joy')
    
    self.params.spatial_scale = rospy.get_param('~spatial_scale', 1.0)
    self.params.ctrl_mode = TurtleParams.VELOCITY_CTRL_MODE
    self.params.accel_throttle = rospy.get_param('~accel_throttle', 100.0)
    self.params.decel_brake = rospy.get_param('~decel_brake', 200.0)
    self.params.coeff_drag = rospy.get_param('~coeff_drag', 1.0/1.6e3)
    self.params.coeff_resistance = rospy.get_param('~coeff_resistance', 30.0*self.params.coeff_drag)
    self.params.mass = rospy.get_param('~mass', 1.0)
    
    self.scale_linear = rospy.get_param('~scale_linear', 200.0)
    self.scale_linear_z = rospy.get_param('~scale_linear_z', 50.0)
    self.scale_angular = rospy.get_param('~scale_angular', 1.6)
    
    set_params_name = '/turtle1/set_params'
    print 'Waiting for service:', set_params_name
    rospy.wait_for_service(set_params_name)
    
    self.set_params_cln = rospy.ServiceProxy(set_params_name, SetTurtleParams)
    self.velocity_pub = rospy.Publisher('/turtle1/command_velocity_xyz', VelocityXYZ, queue_size=10)
    self.drive_pub = rospy.Publisher('/turtle1/command_drive_xyz', DriveXYZ, queue_size=10)
    self.joy_sub = rospy.Subscriber('joy', Joy, self.handle_joy_cb)
    
    self.set_params_cln(new_params=self.params)
    
    rospy.loginfo('Teleop node initialized')
    rospy.loginfo('VELOCITY Ctrls:\n- SELECT: set VELOCITY mode & reset\n- RT: set horizontal velocity\n- LX: set heading\n- RY: set altitude velocity')
    rospy.loginfo('DRIVE Ctrls:\n- START: set DRIVE mode & reset\n- RT: set throttle\n- LT: set brake\n- LX: set heading\n- RY: set altitude velocity')
    rospy.loginfo('Current Mode: VELOCITY')
    
    
  def handle_joy_cb(self, msg):
    if len(msg.axes) != 27 or len(msg.buttons) != 19:
      rospy.logwarn('Ignoring incompatible joy msg with %d axes and %d buttons' % (len(msg.axes), len(msg.buttons)))
      return
      
    LX = msg.axes[0] # left == 1.0
    LY = msg.axes[1] # up == 1.0
    RY = msg.axes[3] # up == 1.0
    LT = msg.axes[12] # idles at 1.0, pushed == -1.0
    RT = msg.axes[13] # idles at 1.0, pushed == -1.0
    Start = msg.buttons[3]
    Select = msg.buttons[0]
    
    RT_pct = (1.0-RT)/2.0
    LT_pct = (1.0-LT)/2.0
    
    if Start:
      if self.params.ctrl_mode != TurtleParams.VELOCITY_CTRL_MODE:
        rospy.loginfo('Current Mode: VELOCITY')
      self.params.ctrl_mode = TurtleParams.VELOCITY_CTRL_MODE
      self.set_params_cln(new_params=self.params)
    elif Select:
      if self.params.ctrl_mode != TurtleParams.DRIVE_CTRL_MODE:
        rospy.loginfo('Current Mode: DRIVE')
      self.params.ctrl_mode = TurtleParams.DRIVE_CTRL_MODE
      self.set_params_cln(new_params=self.params)
      
    if self.params.ctrl_mode == TurtleParams.VELOCITY_CTRL_MODE:
      vmsg = VelocityXYZ()
      vmsg.linear_xy = RT_pct*self.scale_linear
      vmsg.linear_z = RY*self.scale_linear_z
      vmsg.angular_xy = LX*self.scale_angular
      self.velocity_pub.publish(vmsg)
      
    elif self.params.ctrl_mode == TurtleParams.DRIVE_CTRL_MODE:
      dmsg = DriveXYZ()
      dmsg.throttle = RT_pct
      dmsg.brake = LT_pct
      dmsg.linear_z = RY*self.scale_linear_z
      dmsg.angular_xy = LX*self.scale_angular
      self.drive_pub.publish(dmsg)
    
    
  def spin(self):
    rospy.spin()


if __name__ == '__main__':
  try:
    t = Teleop()
    t.spin()
  except rospy.ROSInterruptException:
    pass
