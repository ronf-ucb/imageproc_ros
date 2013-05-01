# simple test to verify that serial connection to IP2.5 is working

import serial

port='/dev/ttySAC1'

ipSerial = serial.Serial(port=port, baudrate=1000000)

print 'Serial Open:', ipSerial.isOpen()

while(1):
    print 'serial in=', ipSerial.read(1)

