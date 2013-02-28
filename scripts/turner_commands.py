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
from imageproc.robot_init import robot_init
import imageproc.shared
import imageproc.run_robot_class

from time import sleep

def handle_command(msg, robotname):
#    print 'desired linear vel ' + str(msg.linear)
#    print 'desired angular rate ' + str(msg.angular)
    sleep(0.025)  # wait to avoid overfilling Basestation queue
    if Robot.robot_ready == True:
#        print '\n des. lin. vel. %6.2f' % msg.linear,
#        print 'des. ang. rate %6.2f' % msg.angular,
#        print 'robot = ' + robotname
        Robot.proceed(msg.linear, msg.angular)

    else: 
        rospy.logerr('%s not ready' % robotname)
        print robotname + 'not ready'
        sleep(0.5)  # may still be initializing



if __name__ == '__main__':
    robotname = 'VelociRoACH1'
    Robot = imageproc.run_robot_class.RunRobot(robotname)
    Robot.robot_ready = False
    rospy.init_node('Turner25')
    rospy.Subscriber('velCmd',
                     turtlesim.msg.Velocity,
                     handle_command,
                     robotname)

    try:
        print 'initializing robot'
        Robot.robot_ready = robot_init()
        print "robot_ready = ", Robot.robot_ready
        
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
