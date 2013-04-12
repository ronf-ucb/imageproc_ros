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
import imageproc.serial_comm
import imageproc.run_robot_class
import time

def stopRobot():
    serial.stop()
    robot.stop()

def clamp(value, minVal, maxVal):
    return max(minVal, min(maxVal, value)) 
    
if __name__ == '__main__':
    rospy.init_node('teleop')

    robotname = 'VelociRoACH1'

    device = rospy.get_param('~device', '/dev/ttySAC1')
    print "device=" + str(device)

    try:
        print 'initializing robot'
        serial = imageproc.serial_comm.SerialComm(device)
        serial.start()
        robot = imageproc.run_robot_class.RunRobot(robotname, serial)
        print 'starting robot'
        robot.start()
    except rospy.ROSInterruptException:
        pass

        # add default value

    rospy.Subscriber('cmd_vel',
                     geometry_msgs.msg.Twist,
                     robot.callback_command,
                     robotname,
                     1)  # queue size 1
    
    rospy.on_shutdown(stopRobot)
    rospy.spin()
