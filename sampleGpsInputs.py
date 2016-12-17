#!/usr/bin/python
import argparse
import serial
from time import sleep

parser = argparse.ArgumentParser(description='Send sample gps data at regular intervals.')
parser.add_argument('-p', action='store', dest='port', default='/dev/ttyACM0')
parser.add_argument('-b', action='store', dest='baudRate', type=int, default=9600)
parser.add_argument('-s', action='store', dest='delay', type=int, default=2)
args = parser.parse_args()

if __name__ == "__main__":
    ser = serial.Serial(args.port, args.baudRate, timeout=10)
    ser.flush()
    while True:
        ser.write('$GPGGA,043955.000,4432.2213,N,12315.3913,W,1,08,0.96,71.8,M,-21.0,M,,*68\n')
        line = ser.readline()[:-2]
        line = line.split(',')
        print line
        # sleep(2)
        # For some reason sleep hangs the program every ~30 seconds.  Idk why yet.

'''
eg3. $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
1    = UTC of Position
2    = Latitude
3    = N or S
4    = Longitude
5    = E or W
6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
7    = Number of satellites in use [not those in view]
8    = Horizontal dilution of position
9    = Antenna altitude above/below mean sea level (geoid)
10   = Meters  (Antenna height unit)
11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
       mean sea level.  -=geoid is below WGS-84 ellipsoid)
12   = Meters  (Units of geoidal separation)
13   = Age in seconds since last update from diff. reference station
14   = Diff. reference station ID#
15   = Checksum
'''
