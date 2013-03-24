#!/usr/bin/env python
import roslib; roslib.load_manifest('imageproc_ros')

import rospy
import serial
import sys
import math
import random
import time

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

'''
provides a .05 second heartbeat when the robot is supposed to be operating
can be enabled or disabled with boolean messages at /robotEnabled
'''

BUTTON_NUM = 5
POLL_TIME = 0.05

def main():
    global pub
    global robotEnabled
    robotEnabled = False
    rospy.loginfo("starting heartbeat")
    rospy.init_node('heartbeat')
    pub = rospy.Publisher("heartbeat", UInt64)
    rospy.Subscriber("/joy", Joy, joystickChanged)

    countNumber = 0

    while (not rospy.is_shutdown()):
        if robotEnabled:
            pub.publish(countNumber)
            countNumber = countNumber + 1
        rospy.sleep(POLL_TIME)

def joystickChanged(data):
    global robotEnabled
    if(data.buttons[BUTTON_NUM]):
        robotEnabled = True
    else:
        robotEnabled = False

main()
