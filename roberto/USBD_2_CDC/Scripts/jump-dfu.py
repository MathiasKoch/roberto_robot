#!/usr/bin/env python

import serial
import sys
import time
import os.path

def exists(path):
    """Test whether a path exists.  Returns False for broken symbolic links"""
    try:
        os.stat(path)
    except OSError:
        return False
    return True


baudRate1 = 230400
baudRate2 = 38400
neutralBaudRate = 9600


if len(sys.argv) > 1:
	portName = sys.argv[1]
else:
	portName = "/dev/ttyACM0"

if exists(portName):
	
	ser = serial.Serial(portName, baudRate1)
	ser.close()

	ser = serial.Serial(portName, baudRate2)
	ser.close()

	ser = serial.Serial(portName, neutralBaudRate)
	ser.close()

	time.sleep(1)