#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Pose

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Robot!
---------------------------
Moving axis:
   u    <-axis3->    o
   j    <-axis2->    l
   m    <-axis1->    .

a : home

CTRL-C to quit
"""

moveBindings = {
        'u':(0,0,1,False),
        'o':(0,0,-1,False),
        'j':(0,1,0,False),
        'l':(0,-1,0,False),
        'm':(1,0,0,False),
        '.':(-1,0,0,False),
        'a':(0,0,0,True)
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('bump_axis', Pose, queue_size = 1)
        self.axis1 = 0
        self.axis2 = 0
        self.axis3 = 0
        self.home = False
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, axis1, axis2, axis3, home):
        self.condition.acquire()
        self.axis1 = axis1
        self.axis2 = axis2
        self.axis3 = axis3
        self.home = home
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, False)
        self.join()

    def run(self):
        command = Pose()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            command.position.x = self.axis1
            command.position.y = self.axis2
            command.position.z  = self.axis3
            command.orientation.w = self.home

            self.condition.release()

            # Publish.
            self.publisher.publish(command)

        # Publish stop message when thread exits.
            command.position.x = self.axis1
            command.position.y = self.axis2,
            command.position.z  = self.axis3
            command.orientation.w = self.home
        self.publisher.publish(command)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_bump_keyboard')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    axis1 = 0
    axis2 = 0
    axis3 = 0
    home = 0
    status = 0

    try:
        #pub_thread.wait_for_subscribers()
        pub_thread.update(axis1, axis2, axis3, home)

        print(msg)
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                axis1 = moveBindings[key][0]
                axis2 = moveBindings[key][1]
                axis3 = moveBindings[key][2]
                home = moveBindings[key][3]

                print (axis1, axis2, axis3, home)
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and axis1 == 0 and axis2 == 0 and axis3 == 0 and home == False:
                    continue
                axis1 = 0
                axis2 = 0
                axis3 = 0
                home = False
                print (axis1, axis2, axis3, home)
                if (key == '\x03'):
                    break
 
            pub_thread.update(axis1, axis2, axis3, home)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)