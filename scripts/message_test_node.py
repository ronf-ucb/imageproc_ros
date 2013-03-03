#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# - Neither the name of the University of California, Berkeley nor the names
# of its contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# R. Fearing Feb. 2013
# test ROS sensor messages 

import sys
# sys.path.append('/opt/ros/groovy/lib/python2.7/dist-packages')
import roslib
roslib.load_manifest('Turner25')
import rospy
import rosgraph
import turtlesim.msg
import sensor_msgs.msg

smsg = sensor_msgs.msg.JointState()

def handle_command(msg, robotname):
    print 'desired linear vel ' + str(msg.linear)
    print 'desired angular rate ' + str(msg.angular)
    print 'robot = ' + robotname

    smsg.header.seq = 100
    smsg.header.stamp.secs = 3.3
    smsg.header.stamp.nsecs = 2357786

    smsg.name = [ 'left', 'right']
    smsg.position = [0.3, 0.7]            # leg encoder count
    smsg.velocity = [1.35, 2.22]          # motor back emf
    smsg.effort = [1024, 2048]            # PWM command
    robot_state_pub.publish(smsg)

# could monitor telem data and send message of current state?


if __name__ == '__main__':
    rospy.init_node('SensorMsg')
# add default value
    robotname = 'VelociRoACH1'
    rospy.Subscriber('velCmd',
                     turtlesim.msg.Velocity,
                     handle_command,
                     robotname)
    robot_state_pub = rospy.Publisher('robotState', sensor_msgs.msg.JointState)
    try:
        print 'initializing node'
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
