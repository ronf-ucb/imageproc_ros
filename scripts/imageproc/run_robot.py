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

# R. Fearing Feb 1, 2013
# basic send command to run robot
# will add telemetry later

import robot_init
from robot_init import *

LEG_VELOCITY = 2.0 # maximum leg m/sec

def setThrust(throttle0, throttle1, duration):
    thrust = [throttle0, throttle1, duration]
    xb_send(0, command.SET_THRUST, pack("3h",*thrust))
    print "cmdSetThrust " + str(thrust)


# run legs in closed loop, with different number of left/right steps

def setThrustClosedLoop(leftTime,rightTime):
    thrust = [throttle[0], leftTime, throttle[1], rightTime, 0]
    xb_send(0, command.SET_THRUST_CLOSED_LOOP, pack('5h',*thrust))
#    print "Throttle[0,1] = ",throttle[0],throttle[1],\
#          "left", leftTime,"right", rightTime

# get one packet of PID data from robot
def getPIDdata():
    count = 0
    shared.pkts = 0   # reset packet count
    dummy_data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    # data format '=LLll'+13*'h' 
    shared.imudata = [] #reset stored data
    xb_send(0, command.GET_PID_TELEMETRY, pack('h',0))
    time.sleep(0.05)
    while shared.pkts == 0:
        print "Retry after 0.5 seconds. Got only %d packets" %shared.pkts
        time.sleep(0.5)
        xb_send(0, command.GET_PID_TELEMETRY, pack('h',0))
        count = count + 1
        if count > 10:
            print 'no return packet'
            shared.imudata.append(dummy_data) # use dummy data
            break   
    data = shared.imudata[0]  # convert string list to numbers
  #  print 'index =', data[0]
  #  print 'time = ', data[1]    # time is in microseconds
  #  print 'mpos=', data[2:4]
  #  print 'pwm=',data[4:6]
  #  print 'imu=',data[6:13]
  #  print 'emf=',data[13:16]
 

# execute move command
# initial - 2 steps straight, 2L+1R for right turn, 1L+2R for left turn
# discrete approximation V_R = V_n + \omega / 2
# V_L = V_n - \omega / 2
# initial calculation assuming no slip - times in milliseconds
def proceed(vel, turn_rate):
    leftTime = int(4*cycle * (vel - turn_rate/2) / LEG_VELOCITY)
    rightTime = int(4 * cycle * (vel + turn_rate/2) / LEG_VELOCITY)

    # probably should normalize, but can at least bound leg run time
    leftTime=max(0,leftTime)
    leftTime=min(4*cycle,leftTime)
    rightTime=max(0,rightTime)
    rightTime=min(4*cycle,rightTime)

    print 'setting run time left=%d  right=%d' %(leftTime, rightTime)
    getPIDdata()
    data = shared.imudata[0]
    currentTime = time.time()   # time in seconds, floating point
    endTime = currentTime + (4*cycle)/1000 # 4 stride motion segments
    setThrustClosedLoop(leftTime, rightTime)
# get telemetry data while closed loop is running
# can't trust robot time - need to have python timer as well
    while(currentTime < endTime):
#       time.sleep(0.1) # sample data every 0.1 sec
        getPIDdata()  # delay is in getPIDdata()
        data = shared.imudata[0]
        currentTime = time.time()  # time in milliseconds
        print 'index =', data[0],'currentTime=',data[1]/1000
   

count = 300 # 300 Hz sampling in steering = 1 sec

def proceed1():
    global duration, count, delay, throttle
    thrust = [throttle[0], duration, throttle[1], duration, 0]
    if telemetry:
        xb_send(0, command.ERASE_SECTORS, pack('h',0))
        print "started erase, 3 second dwell"
        time.sleep(3)
        start = 0   # two byte start time to record
        skip = 0    # store every other sample if = 1
        temp=[count,start,skip]
        print 'temp =',temp,'\n'
        raw_input("Press any key to send StartTelem...")
        xb_send(0, command.START_TELEM, pack('3h',*temp))
        time.sleep(0.1)
    xb_send(0, command.SET_THRUST_CLOSED_LOOP, pack('5h',*thrust))
    print "Throttle = ",throttle,"duration =", duration
    time.sleep(0.1)
    if telemetry:
        flashReadback()

def flashReadback():
    global count, dataFileName
    raw_input("Press any key to start readback ...")
    print "started readback"
    shared.imudata = []  # reset imudata structure
    shared.pkts = 0  # reset packet count???
    xb_send(0, command.FLASH_READBACK, pack('=h',count))
    time.sleep(delay*count + 7)
    while shared.pkts != count:
        print "\n Retry after 10 seconds. Got only %d packets" %shared.pkts
        time.sleep(10)
        shared.imudata = []
        shared.pkts = 0
        xb_send(0, command.FLASH_READBACK, pack('=h',count))
        time.sleep(delay*count + 7)
        if shared.pkts > count:
            print "too many packets"
            break
        if shared.pkts < count:
            print "\n too few packets",str(shared.pkts)
            break
    print "readback done"
# While waiting, write parameters to start of file
    writeFileHeader(dataFileName)     
    fileout = open(dataFileName, 'a')
    np.savetxt(fileout , np.array(shared.imudata), '%d', delimiter = ',')
    fileout.close()
    print "data saved to ",dataFileName


        
def writeFileHeader(dataFileName):
    fileout = open(dataFileName,'w')
    #write out parameters in format which can be imported to Excel
    today = time.localtime()
    date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
    date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
    fileout.write('"Data file recorded ' + date + '"\n')
    fileout.write('"%  keyboard_telem with hall effect "\n')
    fileout.write('"%  motorgains    = ' + repr(motorgains) + '\n')
    fileout.write('"%  delta         = ' +repr(delta) + '"\n')
    fileout.write('"%  intervals     = ' +repr(intervals) + '"\n')
    fileout.write('"% Columns: "\n')
    # order for wiring on RF Turner
    fileout.write('"% time | LPos| RPos | LPWM | RPWM | GyroX | GryoY | GryoZ | GryoZAvg | AX | AY | AZ | RBEMF | LBEMF | VBAT "\n')
 #   fileout.write('"% time | Rlegs | Llegs | DCL | DCR | GyroX | GryoY | GryoZ | GryoZAvg | AX | AY | AZ | LBEMF | RBEMF | SteerOut"\n')
  #  fileout.write('time, Rlegs, Llegs, DCL, DCR, GyroX, GryoY, GryoZ, GryoZAvg, AX, AY, AZ, LBEMF, RBEMF, SteerOut\n')
    fileout.close()

    

