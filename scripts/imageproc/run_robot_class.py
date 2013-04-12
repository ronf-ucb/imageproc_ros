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
import threading
import time
from lib import command
from struct import pack
#import robot_init
#from robot_init import *
# from turner_commands import publish_data
# import turner_commands.publish_data
import sensor_msgs.msg
PI = 3.1415926536
MPOS_SCALE = 2.0 * PI/ (2**16)

smsg = sensor_msgs.msg.JointState()
imsg = sensor_msgs.msg.Imu()    # IMU message

LEG_VELOCITY = 2.0 # maximum leg m/sec

# TODO(ajc) find a home for this
def clamp(value, minVal, maxVal):
    return max(minVal, min(maxVal, value))


class RunRobot(threading.Thread):
    robot_ready = False
    linear_command = 0.0
    angular_command = 0.0
    data = []   # telemetry packet returned from robot
    runtime = 0.0
    robot_onoff = False
    running = False

    def __init__(self, name, comm):
        threading.Thread.__init__(self)
        self.robotname = name
        self.comm = comm
        print "Robot = ", self.robotname
        self.pub_state = rospy.Publisher('robotState', sensor_msgs.msg.JointState)
        self.pub_gyro = rospy.Publisher('robotGyro', sensor_msgs.msg.Imu)

        self.invertLeft = rospy.get_param('~invertLeft', True)
        self.invertRight = rospy.get_param('~invertRight', True)
        self.minThrottle = rospy.get_param('~minThrottle', -0)
        self.maxThrottle = rospy.get_param('~maxThrottle', 0)
        self.linearGain = rospy.get_param('~linearGain', 2400)
        self.angularGain = rospy.get_param('~angularGain', 2400)

        print "invertLeft=" + str(self.invertLeft)
        print "invertRight=" + str(self.invertRight)
        print "minThrottle=" + str(self.minThrottle)
        print "maxThrottle=" + str(self.maxThrottle)
        print "linearGain=" + str(self.linearGain)
        print "angularGain=" + str(self.angularGain)

# store published command locally to be accessed by velocity sending
    def callback_command(self, msg, robotname):
        self.linear_command = msg.linear.x
        self.angular_command = msg.angular.z
        print 'robot.callback_command'

    def callback_runtime(self, msg):
        self.runtime = msg.data + time.time()  # time in milliseconds
        print 'robot runtime =', msg.data, 'end time =', self.runtime

    def setThrust(self, throttle0, throttle1, duration):
        throttle0 = -throttle0 if self.invertLeft else throttle0
        throttle1 = -throttle1 if self.invertRight else throttle1
        throttle0 = clamp(throttle0, self.minThrottle, self.maxThrottle)
        throttle1 = clamp(throttle1, self.minThrottle, self.maxThrottle)        
        thrust = [throttle0, throttle1, duration]
        self.comm.send_command(0, command.SET_THRUST_OPEN_LOOP, pack("3h",*thrust))
#        self.comm.send_command(0, command.SET_THRUST, pack("3h",*thrust))        
        print "cmdSetThrust " + str(thrust)

    # run legs in closed loop, with different number of left/right steps
    def setThrustClosedLoop(self, leftTime,rightTime):
        thrust = [throttle[0], leftTime, throttle[1], rightTime, 0]
#        self.comm.send_command(0, command.SET_THRUST_CLOSED_LOOP, pack('5h',*thrust))
        self.comm.send_command(0, command.SET_THRUST_OPEN_LOOP, pack('5h',*thrust))        
    #    print "Throttle[0,1] = ",throttle[0],throttle[1],\
    #          "left", leftTime,"right", rightTime


    # get one packet of PID data from robot
    def getPIDdata(self):
        count = 0
        shared.pkts = 0   # reset packet count
        dummy_data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        # data format '=LLll'+13*'h' 
        shared.imudata = [] #reset stored data
        self.comm.send_command(0, command.GET_PID_TELEMETRY, pack('h',0))
        time.sleep(0.1)   # 10 Hz, faster could choke Basestation
        while shared.pkts == 0:
            print "Retry after 0.1 seconds. Got only %d packets" %shared.pkts
            rospy.logerr('getPIDdata: retry GET_PID_TELEMETRY')
            self.comm.send_command(0, command.GET_PID_TELEMETRY, pack('h',0))
            time.sleep(0.1)
            count = count + 1
            if count > 5:
                print 'Killed send_command. No return packet.'
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
        self.running = True
        while self.running:


            left_throttle = self.linearGain * self.linear_command - self.angularGain * self.angular_command
            right_throttle = self.linearGain * self.linear_command + self.angularGain * self.angular_command

            self.setThrust(left_throttle, right_throttle, 100)
           # time.sleep(.01)
            time.sleep(5)

            '''
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
       


'''
    def stop(self):
        self.running = False
        self._Thread__stop()




