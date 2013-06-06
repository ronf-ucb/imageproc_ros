#!/usr/bin/env python
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

# R. Fearing Mar. 2013
# read bag files from Turner controller and put in plain text format
# for plotting

import sys
#sys.path.append('/opt/ros/groovy/lib')
sys.path.append('/opt/ros/groovy/lib/python2.7/dist-packages')
import numpy as np
import time

import roslib
# roslib.load_manifest('Turner25')
import rospy
import rosbag

import turtlesim.msg
vmsg = turtlesim.msg.Velocity

import tf
import geometry_msgs.msg  # for optitrackIndexError: list assignment index out of range


import sensor_msgs.msg  
smsg = sensor_msgs.msg.JointState()
imsg = sensor_msgs.msg.Imu()    # IMU message

topic_list = ['/Velocity','/robotState','/robotGyro', '/optitrack/pose']
dataFileName = 'Data/telemdata.txt'
telemdata = []   # structure to write data to
data = [0] * 20
got_both = False


def write_topic(topic,msg,t):
    global vel_topic_count, telemdata, data, got_both
#    print 'topic =', topic
    if (topic == '/Velocity'):
 #        print 'time = %f' % (t.secs - time0 + t.nsecs/1e9),
#        print 'Velocity.linear', msg.linear, 'Velocity.angular', msg.angular
        data[18] = msg.linear
        data[19] = msg.angular
        
    elif (topic == '/robotState'):
        print 'time = %f' % (t.secs - time0 + t.nsecs/1e9),
        print 'sequence =', msg.header.frame_id,
        print 'position', msg.position[0], msg.position[1]
        data[0] = int(msg.header.frame_id)
        data[1] = (t.secs - time0 + t.nsecs/1e9)
        data[2] = msg.position[0]
        data[3] = msg.position[1]
        data[4] = msg.effort[0] # PWM command
        data[5] = msg.effort[1]
        data[13] = msg.velocity[0] # back emf
        data[14] = msg.velocity[1]
        if got_both == False:
            got_both = True
        else:
            got_both = False
            telemdata.append(data[0:20])  # duplicates entires if use just `data'
        
    elif (topic == '/robotGyro'):
        print 'time = %f' % (t.secs - time0 + t.nsecs/1e9),
        print 'sequence =', msg.header.frame_id, 
        print 'angular velocity', msg.angular_velocity.x, msg.angular_velocity.y
        data[6] = msg.angular_velocity.x
        data[7] = msg.angular_velocity.y 
        data[8] = msg.angular_velocity.z
        data[10] = msg.linear_acceleration.x
        data[11] = msg.linear_acceleration.y 
        data[12] = msg.linear_acceleration.z
        if got_both == False:
            got_both = True
        else:
            got_both = False
            telemdata.append(data[0:20])  # duplicates entires if use just `data'


    elif (topic == '/optitrack/pose'):
       # print 'topic =', topic
        data[15] = msg.transform.translation.x
        data[16] = msg.transform.translation.y
        x = msg.transform.rotation.x
        y = msg.transform.rotation.y
        z = msg.transform.rotation.z
        w = msg.transform.rotation.w
        rot = [x,y,z,w]  # quaternion
        angles = tf.transformations.euler_from_quaternion(rot)
        data[17] = angles[2]
        
    else:
        print 'topic %s not defined' % topic


def save_data():
    writeFileHeader(dataFileName)     
    fileout = open(dataFileName, 'a')
    np.savetxt(fileout , np.array(telemdata), '%7.4f', delimiter = ',')
    fileout.close()
    print "data saved to ",dataFileName

def writeFileHeader(dataFileName):
    fileout = open(dataFileName,'w')
    #write out parameters in format which can be imported to Excel
    today = time.localtime()
    date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
    date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
    fileout.write('"%Data file recorded ' + date + '"\n')
    fileout.write('"% recording topics: ' + str(topic_list) + '"\n')
    fileout.write('"% Columns: "\n')
    # order for wiring on RF Turner
    fileout.write('"% seq0 | time1 | LPos2 | RPos3 | LPWM4 | RPWM5' + \
        '| GyroX6 | GryoY7 | GryoZ8 | GryoZAvg9 | AX10 | AY11 | AZ12 |' + \
        'LEMF13 | REMF14 | optX15 optY16 optAng17 | Vel18 Ang19"\n')
    fileout.close()
    


if __name__ == '__main__':
    print 'sys.argv=', sys.argv
    if len(sys.argv) == 2:
        bagFile = sys.argv[1] 
    else:
        bagFile = raw_input('Enter bagfile name:')
        
#    print 'using bagfile ', bagFile
    
#    bag = rosbag.Bag('../bagfiles/tes2.bag')
    bag = rosbag.Bag(bagFile)
# for topic, msg, t in bag.read_messages():
    count = 0
# read once to get time

    for topic, msg, t in \
        bag.read_messages(topics=topic_list):
        print 'read once to get time %f' %(t.secs + t.nsecs/1e9)
        time0 = t.secs
        write_topic(topic,msg,t)
        break

    for topic, msg, t in bag.read_messages(topics=topic_list):
        write_topic(topic,msg,t)
        count = count + 1
        # look at 50 secs of data at 100 Hz
#        if count > 10000 * len(topic_list):
#            break
    bag.close()
    save_data()


