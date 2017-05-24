#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import math
from math import radians, degrees, sin, cos, atan2, tan
import serial
import serial.tools.list_ports
import requests
import time

class ServoOutput(object):
    ''' Methods to move the tracking servos '''
    def __init__(self):
        self.servoCOM = self.findComPort()
        # Default settings for the servo connection
        self.servoBaud = 9600
        self.servoTimeout = 0.5
        self.moveCommand = 0xFF
        # Servo ranges
        self.minPan = 0
        self.maxPan = 255
        self.minTilt = 70
        self.maxTilt = 123
        # Pan and tilt servos on different channels
        self.panChannel = 1
        self.tiltChannel = 0
        # Create a connection to use later
        self.usb = serial.Serial(
            self.servoCOM,
            baudrate = self.servoBaud,
            timeout = self.servoTimeout
        )


    def __del__(self):
        ''' Cleans up when this object is destroyed '''
        if usb:
            print("Closing servo port.")
            self.usb.close()


    def findComPort(self):
        # find the actual com port of the arduino (windows only?)
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'Pololu Micro Maestro 6-Servo Controller Command Port' in p[1]:
                print("Found servos on ", p[0])
                return p[0]
        self.usb = None
        raise IOError("ERROR: Could not find pololu controller for servos.")


    def configServos(self):
        print("Setting servo configurations.")
        #Set acceleration rate for both servos
        accelCommand = 0x89
        tiltAccel = 1
        panAccel = 1
        setAccel = [accelCommand,self.tiltChannel,tiltAccel,0]
        self.usb.write(setAccel)
        setAccel = [accelCommand,self.panChannel,panAccel,0]
        self.usb.write(setAccel)
        #Set rotation rate for both servos
        speedCommand = 0x87
        tiltSpeed = 1
        panSpeed = 3
        setSpeed = [speedCommand,self.tiltChannel,tiltSpeed,0]
        self.usb.write(setSpeed)
        setSpeed = [speedCommand,self.panChannel,panSpeed,0]
        self.usb.write(setSpeed)


    def moveToCenterPos():
        ''' Send servos to their center position '''
        print("debugging: Move to Center Command Sent via ", self.servoCOM)
        moveTiltServo(127)
        movePanServo(127)


    def moveTiltServo(degrees):
        ''' Move the horizontal servo to a given number of degrees '''
        '''
        Notes:
        Servo can take a character from 0-255 and outputs 0-360 degrees
        centerBear is the magnetometer heading
        servo_min is 0
        servo_max is 254
        panOffset is 0
        output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
        '''
        # 0 + ((254 - 0) / (360 - 0)) * (degrees - 0)
        degInServo = 254.0 / 360 * degrees  # should be equiv to above

        if(degInServo < self.minTilt):
            degInServo = self.minTilt
        elif(degInServo > self.maxTilt):
            degInServo = self.maxTilt
        # Maestro controlled can only take characters, so round to an int
        degInServo = int(round(degInServo))
        print("debugging: Move Tilt: degrees={}, degInServo={}".format(degrees, degInServo))
        moveTilt = [self.moveCommand, self.tiltChannel, chr(degInServo)]
        self.usb.write(degInServo)


    def MSUMoveTiltServo(pos):
        tiltTo = (((0-elevation) - tilt_angle_min) * (servo_max - servo_min) / (tilt_angle_max - tilt_angle_min) + servo_min) + tiltOffset
        degInServo


    def movePanServo(position):
        ''' Move the vertical servo to a given position '''
        print("debugging: Move Pan: ", float(position))
        movePan = [moveCommand,panChannel,chr(255-position)]
        self.usb.write(movePan)


    def degToServo(d):
        ''' Converts 0-360 degrees to 0-255 servo positions '''
        # panTo = ((bearing - (centerBear - 168)) * (servo_max - servo_min) / ((centerBear + 168) - (centerBear - 168)) + servo_min) + (255*panOffset/360)
        # tiltTo = (((0-elevation) - tilt_angle_min) * (servo_max - servo_min) / (tilt_angle_max - tilt_angle_min) + servo_min) + tiltOffset
        #return int(round(d*255/360))

        #remove extra 360s
        if d >= 360:
            d = d % 360
        #convert if it is on the right hemisphere
        if d < 180:
            res = int(round(127 - d*(255.0/360.0))) #subtract from the 0 degree position
        #convert if it is on the left hemisphere
        else:
            d = 360 - d #get the degree position going left from center
            res = int(round(127 + d*(255.0/360.0))) # add from the 0 degree position
        print("angle in servo: ", res)
        return res

if __name__ == '__main__':
    print("ServoOutput is not intended to run by itself.")
