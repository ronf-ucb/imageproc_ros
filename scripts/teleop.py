#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#!/usr/bin/env python

import roslib
roslib.load_manifest('imageproc_ros')
import rospy
import geometry_msgs.msg
from imageproc import serial_comm
pos_gain = 1200   # straight throttle component
turn_gain = 1200   # turn gain component
import time

def handle_command(msg, robotname):
    print 'desired linear vel ' + str(msg.linear.x)
    print 'desired angular rate ' + str(msg.angular.z)
    print 'robot = ' + robotname
    left_throttle = pos_gain * msg.linear.x - turn_gain * msg.angular.z
    right_throttle = pos_gain * msg.linear.x + turn_gain * msg.angular.z
    print 'setting thrust left=%d  right=%d' %(left_throttle, right_throttle)
    serial.setThrust(left_throttle, right_throttle, 100)
    
    
if __name__ == '__main__':
    global serial
    rospy.init_node('imageproc_ros')

    # add default value
    robotname = 'VelociRoACH1'
    rospy.Subscriber('/cmd_vel/input',
                     geometry_msgs.msg.Twist,
                     handle_command,
                     robotname)

    try:
        print 'initializing robot'
        serial = serial_comm.SerialComm("/dev/ttyUSB0")
       #robot_init()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
