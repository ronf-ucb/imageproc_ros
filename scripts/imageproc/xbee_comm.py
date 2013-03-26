import comm


import sys
import numpy as np
from lib import command
from struct import *
import time
from xbee import XBee
import serial
from callbackFunc import xbee_received
import shared

DEST_ADDR = '\x20\x52'

class XbeeComm(comm.Comm):
    """
    A Class for Xbee communication to the imageproc2.5
    """

    running = False

    def __init__(self, port):
        threading.Thread.__init__(self)
        self.ser = serial.Serial(shared.BS_COMPORT, shared.BS_BAUDRATE,timeout=3, rtscts=0)

        # TODO(ajc) look at the callback thing
        self.xb = xbee.XBee(self.ser, callback=xbee_received) 

    def run(self):
        self.running = True
        while self.running:
            self.poll()


    def subscribe(self, function):
        """Registers a callback on command received"""
        raise NotImplementedError("subscribe() not implemented")

    def send_command(self, status, type, data):
    	payload = chr(status) + chr(type) + ''.join(data)

    	# TODO(ajc) look at DEST_ADDR
    	self.xb.tx(dest_addr=DEST_ADDR, data=payload)

    def poll(self):
        print "polling"
        if self.SerialCommState is SerialCommState.Length:
            self.lengthByte = ord(self.ser.read(1))
            self.SerialCommState = SerialCommState.ChLength
            print "length=" + str(self.lengthByte)
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
                print self.data
                self.lengthToGo = self.lengthToGo - 1
            else:
                self.SerialCommState = SerialCommState.Checksum
        elif self.SerialCommState is SerialCommState.Checksum:
            checksum = ord(ser.read(1))
            sum = 0xff
            for c in self.data:
                sum = sum + ord(c)
            sum = sum & 0xff
            if checksum is sum:
                print "read success=" + binascii.hexlify(data)
            self.SerialCommState = SerialCommState.Length



if __name__ == '__main__':
    s = SerialComm("/dev/ttyUSB0")
    s.send_command(0, 0x70, "")
    s.send_command(0, 0x80, "\x00\x04\x00\x04")


