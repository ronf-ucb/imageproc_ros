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

# R. Fearing Feb. 2013  basic commands sent out to robot for initial
# closed loop control through ROS

import roslib
roslib.load_manifest('Turner25')
import rospy
import turtlesim.msg
import sensor_msgs.msg
from imageproc.robot_init import robot_init
# from imageproc.robot_init import telemetry
import imageproc.shared
from imageproc import run_robot
from time import sleep
smsg = sensor_msgs.msg.JointState()

def handle_command(msg, robotname):
#    print 'desired linear vel ' + str(msg.linear)
#    print 'desired angular rate ' + str(msg.angular)
    sleep(0.025)  # wait to avoid overfilling Basestation queue
    if imageproc.shared.robot_ready == True:
        print '\n des. lin. vel. %6.2f' % msg.linear,
        print 'des. ang. rate %6.2f' % msg.angular,
        print 'robot = ' + robotname
        run_robot.proceed(msg.linear, msg.angular)
        data = run_robot.getPIDdata()
        publish_data(data)
    else: 
        print robotname + 'not ready'


def publish_data(data):
    smsg.header.seq = data[0]
    smsg.header.stamp.secs = int(data[1]/1e6) 
    smsg.header.stamp.nsecs = data[1] - 1e6 * smsg.header.stamp.secs

    smsg.name = [ 'left', 'right']
    smsg.position = [data[2], data[3]]            # leg encoder count
    smsg.velocity = [data[13], data[14]]          # motor back emf
    smsg.effort = [data[4], data[5]]            # PWM command
    robot_state_pub.publish(smsg)



if __name__ == '__main__':
#    global robot_ready
    imageproc.shared.robot_ready = False     # don't send commands until ready
    rospy.init_node('Turner25')
# add default value
    robotname = 'VelociRoACH1'
    rospy.Subscriber('velCmd',
                     turtlesim.msg.Velocity,
                     handle_command,
                     robotname)
    robot_state_pub = rospy.Publisher('robotState', sensor_msgs.msg.JointState)
    try:
        print 'initializing robot'
        robot_init()
        print "robot_ready = ", imageproc.shared.robot_ready
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
