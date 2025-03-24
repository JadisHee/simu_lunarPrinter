#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import keyboard

def key_monitor():
    pub = rospy.Publisher('/keyboard/keypress', Int32, queue_size=10)
    rospy.init_node('gazebo_keyboard')
    while not rospy.is_shutdown():
        if keyboard.is_pressed('space'):
            pub.publish(Int32(32))
            rospy.sleep(0.2) # 防抖

if __name__ == '__main__':
    try: key_monitor()
    except rospy.ROSInterruptException: pass