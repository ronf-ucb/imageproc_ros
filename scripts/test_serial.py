# simple test to verify that serial connection to IP2.5 is working
# for cmdWhoAmI

import serial

port='/dev/ttySAC1'

ipSerial = serial.Serial(port=port, baudrate=1000000)

print 'Serial Open:', ipSerial.isOpen()
ipSerial.flushInput()   # clear input buffer if left over data
count = ord(ipSerial.read(1))
print 'count=', hex(count)   # byte count
x = ord(ipSerial.read(1))
print 'count parity=',hex(x)    # ~ byte count
status = ord(ipSerial.read(1))
print 'status=',hex(status)    # status byte
command = ord(ipSerial.read(1))
print 'command=',hex(command)   


if( (count + x) == 255):
    data = ipSerial.read(1)
    for i in range(0,count-6):
        data = data + ipSerial.read(1)
        # print ' ',str(ord(x)),
       
chksum = ord(ipSerial.read(1))
print 'chksum=',hex(chksum)

# print out whole string
print 'data =', data

ipSerial.close()
