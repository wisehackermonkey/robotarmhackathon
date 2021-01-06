#!/usr/bin/env python
#publishes a message containting string "hello world"

import rospy
from std_msgs.msg import String
   
if __name__=='__main__':
    rospy.init_node('Hello_world')
    pub=rospy.Publisher('my_messages', String, queue_size=1)
    rate=rospy.Rate(1) #rate of it will be published in Hz
 
    while not rospy.is_shutdown():
        message = "Hello World!" #try changing the message
        pub.publish(message)  
        rate.sleep()