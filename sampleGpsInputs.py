#!/usr/bin/python
import argparse
import serial
from time import sleep

parser = argparse.ArgumentParser(description='Send sample gps data at regular intervals.')
parser.add_argument('-p', action='store', dest='port', default='/dev/ttyACM0')
parser.add_argument('-b', action='store', dest='baudRate', type=int, default=115200)
parser.add_argument('-s', action='store', dest='delay', type=int, default=2)
args = parser.parse_args()

if __name__ == "__main__":
    ser = serial.Serial(args.port, args.baudRate)
    while(1):
        print "Sending string.."
        ser.write('$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62\n')
        print '  ', ser.readline()
        sleep(args.delay)
