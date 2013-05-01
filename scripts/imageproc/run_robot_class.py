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
# Feb. 27: added class to handle publishing data, etc
# and local state


import sys
#sys.path.append('/opt/ros/groovy/lib/python2.7/dist-packages')
sys.path.append('../') # add path to up one level
import rospy
import robot_init
from robot_init import *
# from turner_commands import publish_data
# import turner_commands.publish_data
import sensor_msgs.msg
import shared
PI = 3.1415926536
MPOS_SCALE = 2.0 * PI/ (2**16)

smsg = sensor_msgs.msg.JointState()
imsg = sensor_msgs.msg.Imu()    # IMU message

LEG_VELOCITY = 2.0 # maximum leg m/sec

class RunRobot:
    robot_ready = False
    linear_command = 0.0
    angular_command = 0.0
    data = []   # telemetry packet returned from robot
    runtime = 0.0
    robot_onoff = False

    def __init__(self, name,comm):
        self.robotname = name
        self.comm = comm
        self.dataFileName = 'Data/imudata.txt'
        self.count = 0       # keep track of packet tries
        self.throttle = [0,0]
        print "Robot = ", self.robotname
        self.pub_state = rospy.Publisher('robotState', sensor_msgs.msg.JointState)
        self.pub_gyro = rospy.Publisher('robotGyro', sensor_msgs.msg.Imu)

        # initialize any needed robot parameters
    def init(self):
        print 'Keyboard test for IP2.5c on linaro April 2013\n'
       
        while(1):
            self.comm.send_command(0, command.WHO_AM_I, "0123456789ABCDEF")
            print 'run_robot test loop. next who am i:'
            x = raw_input()
        time.sleep(0.5)
        self.setGain()
        time.sleep(0.5)  # wait for whoami before sending next command
        self.setVelProfile()
        self.comm.send_command(0, command.ZERO_POS,  "Zero motor")
        print 'RunRobot.init: read motorpos and zero'
        print "RunRobot.init: Done Initializing"
        self.robot_ready = True
        return True

# set robot control gains
    def setGain(self):
        count = 0
        while not(shared.motor_gains_set):
            print "Setting motor gains. Packet:",count
            count = count + 1
            self.comm.send_command(0, command.SET_PID_GAINS, pack('10h',*motorgains))
            time.sleep(2)
            if count > 20:
                print "count exceeded. Exit."
                print "Closing serial"
                self.comm.stop()
                print "Exiting..."
                sys.exit(0)

# set velocity profile
# invert profile for motor 0 for VelociRoACH kinematics
    def setVelProfile(self):
        global intervals, vel
        print "Sending velocity profile"
        print "set points [encoder values]", delta
        print "intervals (ms)",intervals
        print "velocities (delta per ms)",vel
        temp0 = intervals + map(invert,delta) + map(invert,vel) # invert 0
        temp1 = intervals+delta+vel
        temp = temp0 + temp1  # left = right
        self.comm.send_command(0, command.SET_VEL_PROFILE, pack('24h',*temp))
        time.sleep(1)


# store published command locally to be accessed by velocity sending
    def callback_command(self, msg, robotname):
        self.linear_command = msg.linear
        self.angular_command = msg.angular

    def callback_runtime(self, msg):
        self.runtime = msg.data + time.time()  # time in milliseconds
        print 'robot runtime =', msg.data, 'end time =', self.runtime

    def setThrust(self, throttle0, throttle1, duration):
        thrust = [throttle0, throttle1, duration]
        self.comm.send_command(0, command.SET_THRUST, pack("3h",*thrust))
        print "RunRobot.setThrust:setThrust " + str(thrust)

    # run legs in closed loop, with different number of left/right steps
    def setThrustClosedLoop(self, leftTime,rightTime):
        thrust = [throttle[0], leftTime, throttle[1], rightTime, 0]
        self.comm.send_command(0, command.SET_THRUST_CLOSED_LOOP, pack('5h',*thrust))
    #    print "Throttle[0,1] = ",throttle[0],throttle[1],\
    #          "left", leftTime,"right", rightTime


    # get one packet of PID data from robot
    def getPIDdata(self):
        count = 0
        shared.pkts = 0   # reset packet count
        dummy_data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        # data format '=LLll'+13*'h' 
        shared.imudata = [] #reset stored data
        xb_send(0, command.GET_PID_TELEMETRY, pack('h',0))
        time.sleep(0.1)   # 10 Hz, faster could choke Basestation
        while shared.pkts == 0:
            print "Retry after 0.1 seconds. Got only %d packets" %shared.pkts
            rospy.logerr('getPIDdata: retry GET_PID_TELEMETRY')
            self.comm.send_command(0, command.GET_PID_TELEMETRY, pack('h',0))
            time.sleep(0.1)
            count = count + 1
            if count > 5:
                print 'Killed SendCommand. No return packet.'
                rospy.logfatal('getPIDdata: no return packet')
                rospy.signal_shutdown('Killed node. No return packet!')
                shared.imudata.append(dummy_data) # use dummy data
                break   
        self.data = shared.imudata[0]  # convert string list to numbers
        self.publish_state(self.data)

      #  print 'index =', data[0]
      #  print 'time = ', data[1]    # time is in microseconds
      #  print 'mpos=', data[2:4]    # motor position is 64K/rev
      #  print 'pwm=',data[4:6]
      #  print 'imu=',data[6:13]
      #  print 'emf=',data[13:16]

    def publish_state(self, data):
        smsg.header.seq = data[0]   # sequence number is overwritten by publish
        smsg.header.stamp.secs = int(data[1]/1e6)
        smsg.header.stamp.nsecs = data[1] - 1e6 * smsg.header.stamp.secs
        smsg.header.frame_id = str(data[0]) # sequence number from ImageProc

        smsg.name = [ 'left', 'right']
        # leg encoder count in radians:
        smsg.position = [data[2]*MPOS_SCALE, data[3]*MPOS_SCALE]
        smsg.velocity = [data[13], data[14]]          # motor back emf       
        smsg.effort = [data[4], data[5]]            # PWM command            
#        print 'smsgs =', smsg
        self.pub_state.publish(smsg)

        imsg.header.seq = data[0]   # sequence number is overwritten by publish
        imsg.header.stamp.secs = int(data[1]/1e6)
        imsg.header.stamp.nsecs = data[1] - 1e6 * smsg.header.stamp.secs
        imsg.header.frame_id = str(data[0]) # sequence number from ImageProc

        imsg.angular_velocity.x = data[6]
        imsg.angular_velocity.y = data[7]
        imsg.angular_velocity.z = data[8]
        imsg.linear_acceleration.x = data[10]
        imsg.linear_acceleration.y = data[11]
        imsg.linear_acceleration.z = data[12]
        self.pub_gyro.publish(imsg)
  

    # execute move command
    # initial - 4 steps straight, 3L+1R for right turn, 1L+3R for left turn
    # discrete approximation V_R = V_n + \omega / 2
    # V_L = V_n - \omega / 2
    # initial calculation assuming no slip - times in milliseconds
    def run(self):
        while True:
            vel = self.linear_command
            turn_rate = self.angular_command
            leftTime = int(4*cycle * (vel - turn_rate/2) / LEG_VELOCITY)
            rightTime = int(4 * cycle * (vel + turn_rate/2) / LEG_VELOCITY)

        # probably should normalize, but can at least bound leg run time
            leftTime=max(0,leftTime)
            leftTime=min(4*cycle,leftTime)
            rightTime=max(0,rightTime)
            rightTime=min(4*cycle,rightTime)

            print 'setting run time left=%d  right=%d' %(leftTime, rightTime)
            currentTime = time.time()   # time in seconds, floating point
            endTime = currentTime + (4.0*cycle)/1000.0 # 4 stride motion segments
            if currentTime < self.runtime: 
                self.setThrustClosedLoop(leftTime, rightTime)
    # get telemetry data while closed loop is running
    # can't trust robot time - need to have python timer as well
    #    print 'currentTime = %f, endTime = %f' %(currentTime, endTime)
            while(currentTime < endTime):
    #       time.sleep(0.1) # sample data every 0.1 sec
                self.getPIDdata()  # delay is in getPIDdata()
                self.data = shared.imudata[0]
                currentTime = time.time()  # time in milliseconds
#                print 'index =', self.data[0],'currentTime=',self.data[1]/1000
       


