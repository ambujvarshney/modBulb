#!/usr/bin/python

import serial
import sys
import time
import sys
import getopt
import os

def hexdump(src, length=16):
    FILTER = ''.join([(len(repr(chr(x))) == 3) and chr(x) or '.' for x in range(256)])
    lines = []
    for c in xrange(0, len(src), length):
        chars = src[c:c+length]
        hex = ' '.join(["%02x" % x for x in chars])
        printable = ''.join(["%s" % ((x <= 127 and FILTER[x]) or '.') for x in chars])
        lines.append("%04x  %-*s  %s\n" % (c, length*3, hex, printable))
    return ''.join(lines)

#--------------------------------------------------------------------------------------------------
def show_help(path):
    print "%s   [options] device" % path
    print "Serial dump"
    print "-b|--baudrate        Baudrate of the serial port"
    print "-h|--help            Show this help"

#--------------------------------------------------------------------------------------------------
i_baud_rate = 115200
s_device = "/dev/ttyUSB0"

try:
    opts, args = getopt.getopt(sys.argv[1:], "b:h", ["baudrate=", "help"])

except getopt.GetoptError, err:
    print str(err)
    sys.exit(2)
  
for opt, arg in opts:
    if opt in ('-b', '--baudrate'):
        i_baud_rate = int(arg)
    elif opt in ('-h', '--help'):
        show_help(sys.argv[0])
        sys.exit(0)
    else:
        print 'Unknown option'
        sys.exit(1)

if len(args) > 0:
    s_device = str(args[0])
else:
    print "ERROR: No device specified"
    show_help(sys.argv[0])
    sys.exit(1)


o_serial = serial.Serial(baudrate=i_baud_rate, xonxoff=True, rtscts=False, dsrdtr=False)
o_serial.port = s_device

o_serial.rts = False
o_serial.dtr = True

o_serial.open()
print "RTS DTR status : %d %d" % (o_serial.rts, o_serial.dtr)

while 1:
    sys.stdout.write(o_serial.read(1))
