import threading
import comm
import serial
import time
import struct
import binascii
from callbackFunc import xbee_received

from lib import command


class SerialCommState:
        Length, ChLength, Data, Checksum = range(4)

class SerialComm(comm.Comm):
    """
    A Class for communication to the imageproc2.5
    """

    running = False

    SerialCommState = SerialCommState.Length
    lengthToGo = 0
    data = ""
    lengthbyte = 0
    lengthCheck = 0

    def __init__(self, port):
        threading.Thread.__init__(self)
        self.ser = serial.Serial(
            port=port,
            baudrate=1000000)
        self.ser.open()
        print self.ser.isOpen()
        '''
        self.ser.close()
        print self.ser.isOpen()
        '''

    def run(self):
        self.running = True
        while self.running:
            self.poll()


    def subscribe(self, function):
        """Registers a callback on command received"""
        raise NotImplementedError("subscribe() not implemented")

    def send_command(self, status, type, data):
        data = self.form_payload(status, type, data)
        self.ser.write(data)

    def poll(self):
        #print "polling"
        if self.SerialCommState is SerialCommState.Length:
            self.lengthByte = ord(self.ser.read(1))
            self.SerialCommState = SerialCommState.ChLength
            #print "length=" + str(self.lengthByte)
        elif self.SerialCommState is SerialCommState.ChLength:
            self.lengthCheck = ord(self.ser.read(1))
            print self.lengthCheck
            if self.lengthByte + self.lengthCheck is 0xff:
                self.SerialCommState = SerialCommState.Data
                self.lengthToGo = self.lengthByte - 3
            else:
                self.SerialCommState = SerialCommState.ChLength
                self.lengthByte = self.lengthCheck
        elif self.SerialCommState is SerialCommState.Data:
            if self.lengthToGo > 0:
                self.data = self.data + self.ser.read(1)
                #print self.data
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
                print "command status=" + str(ord(self.data[0]))
                print "command type=" + str(ord(self.data[1]))
                receiveddata = struct.unpack('16h', self.data)
                for i in range(2,len(receiveddata)):
                    print "data[" + str(i) + "]=" + str(ord(receiveddata[i]))
                self.data = ""
            self.SerialCommState = SerialCommState.Length

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