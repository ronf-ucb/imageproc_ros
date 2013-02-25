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

# R. Fearing Jan. 31, 2013
# ROS initialization under Ubuntu
# Keyboard control with hall effect sensing and telemetry
# updated for IP2.5 and AMS Hall angle sensor Jan. 2013
# treat encoder angle as 16 bits 0 ... 2pi (really 14 bits)
# 


import sys
import numpy as np
from lib import command
from struct import *
import time
from xbee import XBee
import serial
from callbackFunc import xbee_received
import shared
# from shared import robot_ready

DEST_ADDR = '\x20\x52'
imudata_file_name = 'imudata.txt'
telemetry = True
imudata = []
gainsNotSet = True;
delay = 0.025

###### Operation Flags ####
RESET_ROBOT = False
##########################


# [Kp Ki Kd Kanti-wind ff]
# now uses back emf velocity as d term
#motorgains = [300,0,10,0,50, 300,0,10,0,50]
# try just left motor
motorgains = [400,0,400,0,0, 400,0,400,0,0]
throttle = [0,0]
duration = [200,200]  # length of run
cycle = 100 # ms for a leg cycle
# velocity profile
# [time intervals for setpoints]
# [position increments at set points]
# [velocity increments]   
delta = [0x4000,0x4000,0x4000,0x4000]  # adds up to 65536 (2 pi)
intervals = [50, 50, 50, 50]  # total 200 ms
vel = [327, 327,327,327]  # = delta/interval

ser = serial.Serial(shared.BS_COMPORT, shared.BS_BAUDRATE,timeout=3, rtscts=0)
xb = XBee(ser, callback = xbee_received)

def xb_send(status, type, data):
    payload = chr(status) + chr(type) + ''.join(data)
    xb.tx(dest_addr = DEST_ADDR, data = payload)

def resetRobot():
    xb_send(0, command.SOFTWARE_RESET, pack('h',0))


        
#set velocity profile
# invert profile for motor 0 for VelociRoACH kinematics
def setVelProfile():
    global intervals, vel
    print "Sending velocity profile"
    print "set points [encoder values]", delta
    print "intervals (ms)",intervals
    print "velocities (delta per ms)",vel
    temp0 = intervals + map(invert,delta) + map(invert,vel) # invert 0
    temp1 = intervals+delta+vel
    temp = temp0 + temp1  # left = right
    xb_send(0, command.SET_VEL_PROFILE, pack('24h',*temp))
    time.sleep(1)
    

def invert(x):
    return (-x)

# set robot control gains
def setGain():
    count = 0
    while not(shared.motor_gains_set):
        print "Setting motor gains. Packet:",count
        count = count + 1
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        time.sleep(2)
        if count > 20:
            print "count exceeded. Exit."
            print "Halting xb"
            xb.halt()
            print "Closing serial"
            ser.close()
            print "Exiting..."
            sys.exit(0)


            



def robot_init():
    print 'keyboard_telem for IP2.5c Jan. 2013\n'
    global throttle, duration, telemetry, dataFileName
    dataFileName = 'Data/imudata.txt'
    count = 0       # keep track of packet tries
    print "using robot address", hex(256* ord(DEST_ADDR[0])+ ord(DEST_ADDR[1]))
    if RESET_ROBOT:
        print "Resetting robot..."
        resetRobot()
        time.sleep(1)  

    if ser.isOpen():
        print "Serial open. Using port",shared.BS_COMPORT
  
    xb_send(0, command.WHO_AM_I, "Robot Echo")
    setGain()
    time.sleep(0.5)  # wait for whoami before sending next command
    setVelProfile()
    throttle = [0,0]
    tinc = 25;
    # time in milliseconds
    # duration = 5*100 -1  # integer multiple of time steps
    xb_send(0, command.ZERO_POS,  "Zero motor")
    print 'read motorpos and zero'
    print "Done Initializing"
    shared.robot_ready = True
                
        
        

#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        robot_init()
    except KeyboardInterrupt:
        xb.halt()
        ser.close()
    except IOError:
        print "IO Error."
        xb.halt()
        ser.close()
