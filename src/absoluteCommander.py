#!/usr/bin/env python
#publishes absolute motor commands message from a keyboard input

import sys
from select import select
import rospy
from geometry_msgs.msg import Pose


def commander():
  timeout = 60
  rospy.init_node('motor_commander')
  pub=rospy.Publisher('move_axis_absolute', Pose, queue_size=1)
  rate=rospy.Rate(1)

  while not rospy.is_shutdown():
    print "\n\nEnter a relative motion command as 'axis1,axis2,axis3'.\nUse '0' to not move an axis.\nUse '999' on axis 1 to begin homing:"
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
      x = [sys.stdin.readline()]
      command_data = [int(i) for i in x[0].split(',')]
      command = Pose()
      command.position.x = command_data[0]
      command.position.y = command_data[1]
      command.position.z = command_data[2]
      print "\nRelative motion command recieved: "
      print command_data
      pub.publish(command)
    else:
      print "No input. Moving on..."
    rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass