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
roslib.load_manifest('Turner25')
import rospy
import turtlesim.msg
from imageproc.robot_init import robot_init
from imageproc.robot_init import telemetry
from imageproc import run_robot
pos_gain = 1200   # straight throttle component
turn_gain = 1200   # turn gain component


def handle_command(msg, robotname):
    print 'desired linear vel ' + str(msg.linear)
    print 'desired angular rate ' + str(msg.angular)
    print 'robot = ' + robotname
    run_robot.proceed(msg.linear, msg.angular)
# could monitor telem data and send message of current state?


if __name__ == '__main__':
    rospy.init_node('Turner25')
# add default value
    robotname = 'VelociRoACH1'
    rospy.Subscriber('velCmd',
                     turtlesim.msg.Velocity,
                     handle_command,
                     robotname)
    try:
        print 'initializing robot'
        robot_init()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
