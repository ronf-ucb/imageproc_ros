# process telemetry file from ROS Turner25 to make easier to plot
#updated for IP2.5 Mar. 2013
import csv

# angle range 16 bits 0 to 2 pi (truncated angle)
def angleEncoder(value):
    temp = (value*2*3.14159)/(2**16)
    return temp


filedata = csv.reader(open('Data/telemdata.txt','rb'), delimiter=',',quotechar='"')
# discard first 4 rows
for i in range(1,5):
    r=filedata.next()
    print '#',r

# seq0 | time1 | LPos2 | RPos3 | LPWM4 | RPWM5' + \
#        '| GyroX6 | GryoY7 | GryoZ8 | GryoZAvg9 | AX10 | AY11 | AZ12 |' + \
#        'LEMF13 | REMF14 | optX15 optY16 optAng17 | Vel18 Ang19"\n')
# for plotting xmgrace uses set s0...s17
# [0] sequence number
# [1] time: seconds (float)
# [2],[3] (s0,s1) angle (radians)
# [4],[5] (s2,s3) PWM 12 bits
# [6],[7,[8],[9],[10],[11],[12] (s4,s5,s6,s7) gyro x,y,z, zavg
# [10],[11],[12] (s8,s9,s10) accelerometer 16 bits
# [13],[14] (s11,s12) back EMF 10 bit A/D
# [15],[16] (s13,s14) optitrack position in m
# [17] (s15) optitrack angle in radians
# [18] (s16) commanded velocity, m/s
# [19] (s17) commanded angular rate, rad/sec

# data printed [1] - [19], drop sequence number
# grace format [1] = tmes axis
# s0, s1: angle (radians)
# s2, s3: PWM

for r in filedata:
    row=map(float,r)

# time + motor position
    for i in range(1,4):
        print " %6.3f" % row[i],

# PWM+gyro+accel data
    for i in range(4,15):
        print " %4d" % row[i],

# optitrack data and commanded velocity
    for i in range(15,20):
        print " %6.3f" % row[i],
    print ' '

    
 
