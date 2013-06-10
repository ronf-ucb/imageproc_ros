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
# Apr. 24, 2013  added serial interface from ajc

import roslib
roslib.load_manifest('Turner25')
import rospy
import pdb
import turtlesim.msg
import sensor_msgs.msg
import std_msgs.msg

# from imageproc.robot_init import robot_init
import imageproc.shared
import imageproc.serial_comm
import imageproc.run_robot_class


def stopRobot():
    serial.stop()
    Robot.stop()


if __name__ == '__main__':
    robotname = 'VelociRoACH1'
    rospy.init_node('Turner25')
    
    device = rospy.get_param('~device', '/dev/ttySAC1')
    # device = '/dev/ttySAC1' # DEBUG
    print "device=" + str(device)

    try:
        print 'initializing robot'
        # pdb.set_trace()
        serial = imageproc.serial_comm.SerialComm(device)
        # serial.start()  -- don't start, will run when it gets published serial topic
        # import pdb; pdb.set_trace()  # if needed to trace during debug
        Robot = imageproc.run_robot_class.RunRobot(robotname, serial)
        Robot.robot_ready = False
        Robot.runtime = 0.0    # initial run time set to 0 sec
        Robot.robot_ready= Robot.init()   # talk to IP, initialize control, etc
        print "Robot.init done."
        # could use thread for Robot.run(), or just start directly
##        print 'starting robot'
##        Robot.start()       # starts background process (from threading library)
    except rospy.ROSInterruptException:
        pass
    
   
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

    print "robot_ready = ", Robot.robot_ready
    Robot.run()       # run robot using callback messages
    rospy.on_shutdown(stopRobot)
    rospy.spin()
