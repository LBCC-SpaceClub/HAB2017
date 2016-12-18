#!/usr/bin/python
import argparse
import serial
import re
from time import sleep

parser = argparse.ArgumentParser(description='Send sample gps data at regular intervals.')
parser.add_argument('-p', action='store', dest='port', default='/dev/ttyACM0')
parser.add_argument('-b', action='store', dest='baudRate', type=int, default=9600)
parser.add_argument('-s', action='store', dest='delay', type=int, default=2)
args = parser.parse_args()

def checkSum(sentence):
    # Credit: http://doschman.blogspot.com/2013/01/calculating-nmea-sentence-checksums.html
    # cksum = sentence[len(sentence) - 2:]
    chksumdata = re.sub("(\n|\r\n)","", sentence[sentence.find("$")+1:sentence.find("*")])
    csum = 0
    for c in chksumdata:
        csum ^= ord(c)
    return hex(csum)[2:]

def gmapsToNMEA(coord):
    # Assumes a gps coordinate in the US.
    coord = abs(coord)
    DDD = int(coord)
    MM = coord%1*60
    cardinal = 'N' if abs(coord) < 90 else 'W'
    return '{0}{1:.4f},{2}'.format(DDD, MM, cardinal)

# use google earth DD.DDDDDD coordinates here:
targetLat = 44.584902
targetLng = -123.258529
# Alt is in meters
targetAlt = 1000

nmea = '$GPGGA,043955.000,{0},{1},1,08,0.96,{2},M,-21.0,M,,*'.format(
    gmapsToNMEA(targetLat), gmapsToNMEA(targetLng), targetAlt
)
nmea += checkSum(nmea)+'\n'
print nmea

if __name__ == "__main__":
    ser = serial.Serial(args.port, args.baudRate, timeout=10)
    ser.flush()
    while True:
        ser.write(nmea)
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
