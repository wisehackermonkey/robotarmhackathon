#!/usr/bin/env python
#publishes relative motor commands message from a keyboard input

import rospy
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':

  rospy.init_node('motor_commander')
  pub=rospy.Publisher('move_axis_relative', Float64MultiArray, queue_size=1)
  rate=rospy.Rate(1)

  while not rospy.is_shutdown():

      print "Enter a relative motion command as 'axis1,axis2,axis3'. Use '0' to not move an axis:"
      user_input = raw_input()
      print "Relative motion command recieved: " + user_input
      x=[user_input]
      command_data = [float(i) for i in x[0].split(',')]
      command = Float64MultiArray()
      command.data = command_data
      pub.publish(command)
      rate.sleep()