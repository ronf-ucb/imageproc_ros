#BS_COMPORT = 'COM4'
BS_COMPORT = '/dev/ttyUSB0'
BS_BAUDRATE = 230400

motor_gains_set = False
steering_gains_set = False
steering_rate_set = False
robot_ready = False

pkts = 0  # count of packets received
telem_index = 0   # count of packets sent by ImageProc since last reset
count2deg = 2000.0/(2**15-1)

imudata = []
statedata = []
dutycycles = []
