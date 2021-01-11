#!/usr/bin/env python
#publishes relative motor commands message from a keyboard input

import rospy
from geometry_msgs.msg import Pose

if __name__ == '__main__':

  rospy.init_node('motor_commander')
  pub=rospy.Publisher('move_axis_relative', Pose, queue_size=1)
  rate=rospy.Rate(1)

  while not rospy.is_shutdown():

      print "Enter a relative motion command as 'axis1,axis2,axis3'. Use '0' to not move an axis:"
      user_input = raw_input()
      print "Relative motion command recieved: " + user_input
      x=[user_input]
      command_data = [int(i) for i in x[0].split(',')]
      command = Pose()
      command.position.x = command_data[0]
      command.position.y = command_data[1]
      command.position.z = command_data[2]

      pub.publish(command)
      rate.sleep()
  rospy.spin()