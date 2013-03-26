# Copyright (c) 2010-2013, Regents of the University of California
# All rights reserved.
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

# R. Fearing Feb 14, 2014
# publish telemetry data
import sys
sys.path.append('/opt/ros/groovy/lib/python2.7/dist-packages')
import roslib
import rospy

import robot_init
# from robot_init import *
import sensor_msgs.msg

# idx | time | LPos| RPos | LPWM | RPWM | 
# GyroX | GryoY | GryoZ | GryoZAvg | AX | AY | AZ | 
# RBEMF | LBEMF | VBAT | Steer
dummy_data = [100, 0x1000000, 0x2000, -0x2000, 0xfff, 0xfff,\
                  0,0,0,0,0.1,0.2,9.80,\
                  0x1ff,-0x1ff,0x2ff,0]

def pub_telem(data):

    print 'index =', data[0]
    print 'time = ', data[1]    # time is in microseconds
    print 'mpos=', data[2:4]
    print 'pwm=',data[4:6]
    print 'gyro=',data[6:10]
    print 'imu=',data[10:13]
    print 'emf=',data[13:16]

    imsg = sensor_msgs.msg.Imu()
    imsg.header.seq = data[0]
    imsg.angular_velocity.x = data[6]
    imsg.angular_velocity.y = data[7]
    imsg.angular_velocity.z = data[8]
    imsg.linear_acceleration.x = data[10]
    imsg.linear_acceleration.y = data[11]
    imsg.linear_acceleration.z = data[12]

    smsg = sensor_msgs.msg.JointState()
    smsg.header.seq = data[0]
    smsg.name = 'TurnerJoints'
    smsg.position = data[2:4]  # motor pos
    smsg.velocity = data[13:15]   # back EMF
    smsg.effort = data[4:6]  # PWM command

    print 'imsg:', imsg
    print 'smsg:', smsg
    



    

