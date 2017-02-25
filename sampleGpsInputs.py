#!/usr/bin/python
# from __future__ import print_function
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
    cardinal = 'N' if coord < 90 else 'W'
    return '{0}{1:07.4f},{2}'.format(DDD, MM, cardinal)

# use google earth DD.DDDDDD coordinates here:
targetLat = 44.546105
targetLng = -123.256989
# Alt is in meters
targetAlt = 1000

nmea = '$GPGGA,043955.000,{0},{1},1,08,0.96,{2},M,-21.0,M,,*'.format(
    gmapsToNMEA(targetLat), gmapsToNMEA(targetLng), targetAlt
)
nmea += checkSum(nmea)+'\n'
print 'nmea=',nmea

if __name__ == "__main__":
    ser = serial.Serial(args.port, args.baudRate, timeout=10)
    ser.flush()
    print "pLat, pLng, pAlt, sLat, sLng, sAlt, course, cardinal, surfaceDist, vertAngle"
    while True:
        ser.write(nmea)
        line = ser.readline()[:-2]
        # line = line.split(',')
        print line
        # sleep(2)
        # For some reason sleep hangs the program every ~30 seconds.  Idk why yet.
