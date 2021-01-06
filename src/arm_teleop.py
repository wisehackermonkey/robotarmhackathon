#!/usr/bin/env python
#publishes keyboard teleop commands to control the motor. Also has a homing button

import rospy
from std_msgs.msg import Int16MultiArray
from pynput import keyboard

def on_activate_h():
    print('h pressed')

def on_activate_i():
    print('i pressed')




axis1_command = 0
axis2_command = 0
axis3_command = 0


def teleop_parser(user_input):
    global axis1_command
    global axis2_command
    global axis3_command

    if user_input == ' ':
        command_data = [0,0,0]
    #elif user_input = 
    

    command_data = [axis1_command, axis2_command, axis3_command]
    command = Int16MultiArray()
    command.data = command_data

    return command

if __name__ == '__main__':

  rospy.init_node('motor_commander')
  pub=rospy.Publisher('bump_axis', Int16MultiArray, queue_size=1)
  rate=rospy.Rate(10)
  print "Enter commands to teleoperate the robot in bump mode. \n\
  Commands are as follows:\n\n\
  axis 1: h <--CCW---------CW--> k\n\n\
  axis 2: u <--CCW---------CW--> m\n\n\
  axis 3: y <--CCW---------CW--> i\n\n\i
  stop all: space\n\n\
  home all axis: a\n\n"


  while not rospy.is_shutdown():
      """       print "Enter a command:"
      user_input = raw_input()
      print "Relative motion command recieved: " + user_input
      x=[user_input] """

      with keyboard.GlobalHotKeys({
        'h': on_activate_h,
        'i': on_activate_i}) as h:
        h.join()

      command = teleop_parser(user_input)
      pub.publish(command)
      rate.sleep()