import threading
import comm
import serial
import time
import struct
import binascii
import rospy
from callbackFuncSerial import serial_received

from lib import command


class SerialCommState:
        Length, ChLength, Data, Checksum = range(4)

class SerialComm(comm.Comm):
    """
    A Class for communication to the imageproc2.5
    """

    running = False

    SerialCommState = SerialCommState.Length  # get length byte
    lengthToGo = 0
    data = ""
    lengthbyte = 0
    lengthCheck = 0
    SerialSuccess = False

    def __init__(self, port):
        # no threading version - call with scheduler
        # threading.Thread.__init__(self)
        self.ser = serial.Serial(
            port=port,
            baudrate=1000000)
        self.ser.open()
        self.ser.flushInput()   # discard input from previous runs
        print 'Serial Open:', self.ser.isOpen()
        '''
        self.ser.close()
        print self.ser.isOpen()
        '''

    def run(self):
        self.running = True
        # import pdb; pdb.set_trace()   # DEBUG BREAK
        print 'starting polling'
        while self.running:
            self.poll()
        print 'serial polling finished'

    def do_poll(self):
            SerialSuccess = False
            while !SerialSuccess:
                    self.poll()

    def stop(self):
        self.running = False
        self._Thread__stop()


    def subscribe(self, function):
        """Registers a callback on command received"""
        raise NotImplementedError("subscribe() not implemented")

    def send_command(self, status, type, data):
        data = self.form_payload(status, type, data)
        self.ser.write(data)

# should wait until gets consecutive complementary bytes
# should also go back to wait state if exceeds max packet length or has other error
    def poll(self):
        # print "polling"
        time1 = rospy.get_time()
        if self.SerialCommState is SerialCommState.Length:
            self.lengthByte = ord(self.ser.read(1))
            self.SerialCommState = SerialCommState.ChLength
            time1 = rospy.get_time()
            # print "length=" + hex(self.lengthByte),
        elif self.SerialCommState is SerialCommState.ChLength:
            self.lengthCheck = ord(self.ser.read(1))
            # print ' length check =', hex(self.lengthCheck),
            if self.lengthByte + self.lengthCheck is 0xff:
                self.SerialCommState = SerialCommState.Data
                self.lengthToGo = self.lengthByte - 3
            else:
                self.SerialCommState = SerialCommState.ChLength
                self.lengthByte = self.lengthCheck # check if last byte received is length byte
        elif self.SerialCommState is SerialCommState.Data:
            if self.lengthToGo > 0:
                self.data = self.data + self.ser.read(1)
                # print self.data
                self.lengthToGo = self.lengthToGo - 1
            else:
                self.SerialCommState = SerialCommState.Checksum
        elif self.SerialCommState is SerialCommState.Checksum:
            checksum = ord(self.ser.read(1))
            sum = 0xff
            for c in self.data:
                sum = sum + ord(c)
            sum = sum & 0xff
            if checksum is sum:
                #print "read success=" + binascii.hexlify(self.data)
                # print "command status=" + str(ord(self.data[0])),
                # print "command type=" + hex(ord(self.data[1]))
                SerialSuccess = True
                serial_received(self.data)  # process serial packet
                # receiveddata = struct.unpack('16h', self.data)
                # print 'Checksum OK. checksum =', hex(checksum), ' sum =', hex(sum)
                self.data = ""        
                self.SerialCommState = SerialCommState.Length # ready for next packet
                time2 = rospy.get_time()
                print "serial poll time", str(time1) + " " + str(time2 - time1) 
            else:
                print 'Checksum error. checksum =', hex(checksum), ' sum =', hex(sum)
                SerialSuccess = False
                self.data = ""
                self.lengthByte = checksum # check if last byte received is length byte
                self.SerialCommState = SerialCommState.ChLength # ready for next packet
            
##                for i in range(2,len(receiveddata)):
##                    print "data[" + str(i) + "]=" + str(ord(receiveddata[i]))

        
    def form_payload(self, status, type, data):
        payload = chr(status) + chr(type) + ''.join(data)
        return self.serial_payload(payload)


    def serial_payload(self, data):
        """length, bitwise flip length, data, checksum"""

        # + 3 for the length, ~length, and checksum
        length = len(data) + 3
        datalengthCompliment = ~length if ~length >= 0 else ~length + 256
        payload = chr(length) + chr(datalengthCompliment) + ''.join(data)
        sum = 0xff
        for c in data:
            sum = sum + ord(c)
        sum  = sum & 0xff
        return payload + chr(sum)

    def setThrust(self, throttle0, throttle1, duration):
        thrust = [throttle0, throttle1, duration]
        self.send_command(0, command.SET_THRUST_OPEN_LOOP, struct.pack("3h",*thrust))
        print "cmdSetThrust " + str(thrust)


if __name__ == '__main__':
    s = SerialComm("/dev/ttyUSB2")

#    s.setThrust(0xf80, 0xf80, 500)
    #s.setThrust(1200, 1200, 400)
    '''
    s.send_command(0, 0x72, struct.pack('h', 0))
    s.start()
    while 1:
        s.send_command(0, 0x72, struct.pack('h', 0))
        time.sleep(.1)
    '''
    #to send to bird
    #s.send_command(0, 0x21,  pack('B', 1))
    s.send_command(0, 0x71, chr(0) + chr(0x21) + ''.join(struct.pack('B', 2)))
    s.send_command(0, 0x71, chr(0) + chr(0x25) + ''.join(struct.pack('3f', 0.0,1.0,0.0)))
    #s.send_command(0, 0x80, "\x00\x04\x00\x04")
