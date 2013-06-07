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
from imageproc.callbackFuncSerial import serial_received
import pdb

StateLength = 0
StateChLength = 1
StateData = 2
StateChecksum = 3
SerialCommState = StateLength
lengthToGo = 0
data = ""
lengthByte = 0
lengthCheck = 0

def handle_serial_receive(msg):
    global state_initialized
    print "msg=", msg
    pkt_time = msg.stamp
    print "secs "+ str(msg.stamp.secs) + " nsecs: " + str(msg.stamp.nsecs)
    serial_data = msg.data
    print "serial data =" + msg.data
    print "length of data =" + str(len(serial_data))
    pdb.set_trace()
    for i in range(0,len(serial_data)):
        handle_byte(serial_data[i])

        
    
# should wait until gets consecutive complementary bytes
# should also go back to wait state if exceeds max packet length or has other error
def handle_byte(byte):
    global SerialCommState, StateLength, StateChLength, StateData, StateChecksum
    global  lengthToGo, data, lengthByte, lengthCheck
    if SerialCommState == StateLength:
        lengthByte = ord(byte)
        SerialCommState = StateChLength
    elif SerialCommState == StateChLength:
        lengthCheck = ord(byte)
        # print ' length check =', hex(lengthCheck),
        if lengthByte + lengthCheck is 0xff:
            SerialCommState = StateData
            lengthToGo = lengthByte - 3
        else:
            SerialCommState = StateChLength
            lengthByte = lengthCheck # check if last byte received is length byte
    elif SerialCommState == StateData:
        if lengthToGo > 0:
            data = data + byte
            # print data
            lengthToGo = lengthToGo - 1
        else:
            SerialCommState = StateChecksum
    elif SerialCommState == StateChecksum:
        checksum = byte
        sum = 0xff
        for c in data:
            sum = sum + ord(c)
        sum = sum & 0xff
        if checksum is sum:
            #print "read success=" + binascii.hexlify(data)
            # print "command status=" + str(ord(data[0])),
            # print "command type=" + hex(ord(data[1]))
            SerialSuccess = True
            serial_received(data)  # process serial packet
            # receiveddata = struct.unpack('16h', data)
            # print 'Checksum OK. checksum =', hex(checksum), ' sum =', hex(sum)
            data = ""        
            SerialCommState = StateLength # ready for next packet
        else:
            print 'Checksum error. checksum =', hex(checksum), ' sum =', hex(sum)
            SerialSuccess = False
            data = ""
            lengthByte = checksum # check if last byte received is length byte
            SerialCommState = StateChLength # ready for next packet




    
    
if __name__ == '__main__':
    rospy.init_node('SerialPacket')
# add default value
    SerialCommState = StateLength
   
    rospy.Subscriber('/serial_node/serial_receive',
                     asrl_sensor_msgs.msg.SerialData,
                     handle_serial_receive, None, 1)  # queue size 1
    pub= rospy.Publisher('/serial_node/serial_send', asrl_sensor_msgs.msg.SerialData)
    print "serial_parse node initialized"
    rospy.spin()
