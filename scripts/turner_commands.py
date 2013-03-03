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
# Feb. 28, 2013  changed to using Robot 

import roslib
roslib.load_manifest('Turner25')
import rospy
import turtlesim.msg
import sensor_msgs.msg
import std_msgs.msg
from imageproc.robot_init import robot_init
import imageproc.shared
import imageproc.run_robot_class


if __name__ == '__main__':
    robotname = 'VelociRoACH1'
    Robot = imageproc.run_robot_class.RunRobot(robotname)
    Robot.robot_ready = False
    Robot.runtime = 0.0    # initial run time set to 0 sec
    rospy.init_node('Turner25')
# topic for commanded leg velocities from controller node
    rospy.Subscriber('velCmd',
                     turtlesim.msg.Velocity,
                     Robot.callback_command,
#                     handle_command,
                     robotname)
# topic for starting and starting robot
    rospy.Subscriber('RunTime', 
                     std_msgs.msg.Float32,
                     Robot.callback_runtime)

    try:
        print 'initializing robot'
        Robot.robot_ready = robot_init()
        print "robot_ready = ", Robot.robot_ready
        Robot.run()       # run robot using callback messages
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
