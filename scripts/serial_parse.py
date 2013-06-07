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
# R. Fearing June 2013
# send and receive serial messages using asrl_comms

import roslib
roslib.load_manifest('serial')
import rospy
import math
import tf
import asrl_sensor_msgs.msg


def handle_turner_pose(msg, robotname):
    global state_initialized
#    robotname = 'roach'
#    print "msg=", msg
    getState(msg)
    if state_initialized == False:
        initialize_target()
        state_initialized = True
        return  # don't publish on first message
# control law - use bicycle steering
# PD control on longitudinal = x
# P control on lateral = y with d equiv from orientation
    x = state[0]
    y = state[1]
    theta = state[2]
    vel = kpx * (target[0] - x)
    turn_rate = kpy * (target[1] - y) + kd * (target[2] - theta)
# controller should apply sanity checks
    vel = min(MAX_VEL, vel)
    vel = max(-MAX_VEL, vel)
    turn_rate = min(MAX_TURN, turn_rate)
    turn_rate = max(-MAX_TURN,turn_rate)
    print 'cmd speed m/s %6.2f' % vel, ' cmd turn rad/sec = %6.2f' % turn_rate
    pub.publish(turtlesim.msg.Velocity(vel,turn_rate))

# put initial target location a bit in front of robot by about 0.5 m
def initialize_target():
    target[0] = state[0] + 0.0  # x_o
    target[1] = state[1] # y_o
    target[2] = 0.0  # orientation should be straight

def getState(msg):
    global state, old_state
    old_state = state
    x_est = msg.transform.translation.x
    y_est = msg.transform.translation.y
    x = msg.transform.rotation.x
    y = msg.transform.rotation.y
    z = msg.transform.rotation.z
    w = msg.transform.rotation.w
    rot = [x,y,z,w]  # quaternion
#    print 'quaternion=', rot
    angles = tf.transformations.euler_from_quaternion(rot)
#    print 'angles = ', angles
    print 'state: x_est= %8.3f' % x_est,' y_est=%8.3f' % y_est, ' theta=%8.3f' % angles[2]
    state = [x_est, y_est, angles[2]]


if __name__ == '__main__':
    rospy.init_node('SerialParse')
# add default value
    
    rospy.Subscriber('/serial_node/serial_receive',
                     asrl_sensor_msgs.msg.SerialData,
                     handle_turner_pose, robotname,
                     1)  # queue size 1
    pub= rospy.Publisher('velCMD', asrlturtlesim.msg.Velocity)
    rospy.spin()
