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
# should get pose message from optitrack, and calculate commanded
# robot velocity to get to origin 

import roslib
roslib.load_manifest('Turner25')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.msg

# state variables
state_initialized = False
target = [0,0,0] # desired position for robot
state = [0, 0, 0]
old_state = [0,0,0]
kpx = 2.0   # should be m/s per meter longitudinal error
kpy = 4.0   # should be rad/sec turn per meter lateral error
kd = 1.0  # should be rad/sec turn per rad orientation error
ref = [0,0,0] # reference position to track, default to origin
MAX_VEL = 2.0 # m/s
MAX_TURN = 3.0 # rad/sec
# need Pose and TransformStamped

def handle_turner_pose(msg, robotname):
    global state_initialized
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
    print 'cmd speed m/s', vel, ' cmd turn rad/sec =', turn_rate
    pub.publish(turtlesim.msg.Velocity(vel,turn_rate))

# put initial target location a bit in front of robot by about 0.5 m
def initialize_target():
    target[0] = state[0] + 0.5  # x_o
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
    print 'angles = ', angles
    print 'state: x_est=',x_est,' y_est=', y_est, ' theta=', angles[2]
    state = [x_est, y_est, angles[2]]


if __name__ == '__main__':
    rospy.init_node('Control')
# add default value
    state_initialized = False
    robotname = 'VelociRoACH1'
    rospy.Subscriber('/optitrack/pose',
                     geometry_msgs.msg.TransformStamped,
                     handle_turner_pose,
                     robotname)
    pub= rospy.Publisher('velCMD', turtlesim.msg.Velocity)
    rospy.spin()
