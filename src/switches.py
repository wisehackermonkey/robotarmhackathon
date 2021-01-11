#!/usr/bin/env python
 
import rospy
from std_msgs.msg import String, Int16
 
switch1_val=0
switch2_val=0
switch3_val=0

def switch1_callback(msg):
    global switch1_val
    switch1_val=msg.data
    if switch1_val==1:
        rospy.loginfo('Switch 1 was pressed')
        
def switch2_callback(msg):
    global switch2_val
    switch2_val=msg.data
    if switch2_val==1:
        rospy.loginfo('Switch 2 was pressed')

def switch3_callback(msg):
    global switch3_val
    switch3_val=msg.data
    if switch3_val==1:
        rospy.loginfo('Switch 3 was pressed')
   
if __name__=='__main__':
    rospy.init_node('Switch_multiplex')
   
    sub1=rospy.Subscriber('arduino_switch1', Int16, switch1_callback)
    sub2=rospy.Subscriber('arduino_switch2', Int16, switch2_callback)
    sub3=rospy.Subscriber('arduino_switch3', Int16, switch3_callback)
    
    pub=rospy.Publisher('switch_status', String, queue_size=1)
    rate=rospy.Rate(10)
 
    while not rospy.is_shutdown():
        switch_message = str(switch1_val) + "  " + str(switch2_val) + "  " + str(switch3_val)
        pub.publish(switch_message)
        rate.sleep()
        rospy.spin()