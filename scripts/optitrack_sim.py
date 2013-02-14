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
# simple tool to publish on optitrack/pose topic

import roslib
roslib.load_manifest('Turner25')
import rospy

import tf
import turtlesim.msg
import geometry_msgs.msg

# generate optitrack style type messages from user input of x,y,theta


def optitrack_sim(robotname):
    msg = geometry_msgs.msg.TransformStamped()
#   print 'msg =', msg
    pub = rospy.Publisher('/optitrack/pose',
                           geometry_msgs.msg.TransformStamped)
    while not rospy.is_shutdown():
        print 'enter x,y, theta for robot pose'
        temp = raw_input()
        vec = map(float,temp.split(','))
        x = vec[0]
        y = vec[1]
        theta = vec[2]
#       print 'x,y,theta =', x,y,theta
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
#        print 'msg =', msg
        pub.publish(msg)
# TransformBroadcaster: tf not compatible with geometry_msgs?
#        br = tf.TransformBroadcaster()
#        br.sendTransform((x, y, 0),
#                     tf.transformations.quaternion_from_euler(0, 0, theta),
#                     rospy.Time.now(),
#                     robotname,
#                     "world")

if __name__ == '__main__':
    rospy.init_node('Optitrack_sim')
    robotname = 'VelociRoACH1'
# add default value
    try:
         optitrack_sim(robotname)
    except rospy.ROSInterruptException:   # run until control-C
         pass


