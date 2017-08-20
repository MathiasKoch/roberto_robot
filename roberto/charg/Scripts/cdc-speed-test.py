from __future__ import print_function

import time
import serial
import sys

ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    timeout=100000
)

i = 0;
Bytes = 1000;

#wait until bootloader finished
#time.sleep(2)

if len(sys.argv) > 1:
    if "rx" in sys.argv[1].lower():
        mode = 1
    elif "tx" in sys.argv[1].lower():
        mode = 2
    else:
        mode = 3
else:
    mode = 3


if mode == 1:
    print("Speed testing USB CDC port: %s, for RX speed" % ser.port)
elif mode == 2:
    print("Speed testing USB CDC port: %s, for TX speed" % ser.port)
elif mode == 3:
    print("Speed testing USB CDC port: %s, for RX/TX speed" % ser.port)

err = 0

bulkSize = 26
letters = (chr(ord('a') + x) for x in range(bulkSize))


try:
    if mode != 1:
        t0 = time.time()
        out = 0;
    while i < Bytes:
        if mode == 3:
            # Write chunks of 16 bytes
            for x in letters:
                out += ser.write(x)
            ser.flush()
            ret = ser.read(1)
            ret += ser.read(ser.in_waiting)
            err += sum(c not in letters for c in ret)
            i += len(ret)
        elif mode == 2:
            n = ser.in_waiting
            if(n>0):
                ret = ser.read(n)
                err += sum(c not in "R" for c in ret)
                i += n
        elif mode == 1:
            ser.write(b'TTTTTTTTTTTTTTTT')
            ser.flush()
            i += 16
    if mode != 1:
        t1 = time.time()
        MBps = (Bytes/(t1-t0) * 1e-6)
    else:
        n = ser.in_waiting
        while(n <= 0):
            pass
        result = ser.read(n)
        print(result)
        #MBps = int(ser.read(n))
except KeyboardInterrupt:
    ser.close()
    exit(1)

ser.close()

if mode == 3:
    if out != i:
        print("\r\nSomething went wrong with the bytes!\r\n     Transmited:\t%4d Bytes\r\n     Received:\t\t%4d Bytes\r\n     Lost:\t\t%4d Bytes\r\n" % (out, i, out-i))
    print("Loopback speed: \r\n     %.6f MByte/s\r\nImprovement from 115200 baud UART: \r\n     %.6fMByte/s = %.2f %% \r\nTotal error count: %d" % (MBps, MBps-0.0144, MBps/0.0144*100, err))
elif mode == 2:
    print("Transmit speed: \r\n     %.6f MByte/s\r\nImprovement from 115200 baud UART: \r\n     %.6fMByte/s = %.2f %% \r\nTotal error count: %d" % (MBps, MBps-0.0144, MBps/0.0144*100, err))
#elif mode == 1:
    #print("Receive speed: \r\n     %.6f MByte/s\r\nImprovement from 115200 baud UART: \r\n     %.6fMByte/s = %.2f %% \r\nTotal error count: %d" % (MBps, MBps-0.0144, MBps/0.0144*100, err))
