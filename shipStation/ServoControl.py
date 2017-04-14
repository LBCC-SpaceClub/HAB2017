#!/usr/bin/env python
import math
import serial
import requests
import time

class ServoControl:
    ''' Methods to move the tracking servos '''
    def __init__(self):
        self.servoCOM = self.findComPort()
        if self.servoAttached:
            print "Found servos on ", self.servoCOM
        else:
            print "ERROR: Could not find pololu controller for servos."
            return None
        # Default settings for the servo connection
        servoBaud = 9600
        servoTimeout = 0.5
        # use these to manually tweak the tracking
        self.panOffset = 0     # increase to turn right, decrease to turn left
        self.tiltOffset = 0    # increase to raise, decrease to lower
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
        print "Closing servo port."
        self.usb.close()

    def findComPort(self):
        # find the actual com port of the arduino (windows only?)
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'Pololu Micro Maestro 6-Servo Controller Command Port' in p[1]:
                self.servoAttached = True
                return p[0]
        return None

    def configServos(self):
        print "Setting servo configurations."
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
        print "debugging: Move to Center Command Sent via ", self.servoCOM
        moveTiltServo(127)
        movePanServo(127)

    def moveTiltServo(position):
        ''' Move the horizontal servo to a given position '''
        print "debugging: Move Tilt: ", float(position)
        if(position < self.minTilt):
            moveTilt = [moveCommand,tiltChannel,chr(self.minTilt)]
        elif(position > self.maxTilt):
            moveTilt = [moveCommand,tiltChannel,chr(self.maxTilt)]
        else:
            moveTilt = [moveCommand,tiltChannel,chr(position)]
        self.usb.write(moveTilt)

    def movePanServo(position):
        ''' Move the vertical servo to a given position '''
        print "debugging: Move Pan: ", float(position)
        movePan = [moveCommand,panChannel,chr(255-position)]
        self.usb.write(movePan)

    def findMaxRSSI():
        x=127
        y=127
        movement = 5
        delay = 0.8
        global servo_min, servo_max
        with requests.Session() as session:
            session.get('http://192.168.1.20/login.cgi')  # This line has magic sauce
            data={'uri': '','username': 'ubnt','password': 'ubnt'}
            # Use files trick to post using multipart/form-data encoding.
            r = session.post(
                'http://192.168.1.20/login.cgi',
                files={k: (None, v) for k, v in data.items()},
                verify=False
            )
            baseline = getRSSI(session)
            temp = baseline
            while(True):
                #rssi = getRSSI(session)
                #print "Signal={}, rssi={}".format(signal, rssi)
                print "Starting values: pan={}, tilt={}, baseline={}, RSSI={}.".format(x,y,baseline,getRSSI(session))

                '''
                if(x > servo_min):
                    x -= movement
                    movePanServo(x)
                    time.sleep(delay)
                    temp = getRSSI(session)
                '''
                # Pan left until you can't turn any farther, or the signal fades
                while(x > servo_min and temp >= baseline):
                    x -= movement
                    movePanServo(x)
                    time.sleep(delay)
                    temp = getRSSI(session)
                    print "\tDOWN RSSI: ", temp
                    if(temp > baseline):
                        baseline = temp
                '''
                if(x < servo_max):
                    x += movement
                    movePanServo(x)
                    time.sleep(delay)
                    temp = getRSSI(session)
                '''
                # Pan right until you can't turn any farther, or the signal fades
                while(x < servo_max and temp >= baseline):
                    x += movement
                    movePanServo(x)
                    time.sleep(delay)
                    temp = getRSSI(session)
                    print "\tUP RSSI: ", temp
                    if(temp > baseline):
                        baseline = temp

                #time.sleep(0.1)
                baseline -= 0.1

    def getRSSI(session):
        ''' Read the signal strength from the ubiquity modem '''
        #https://community.ubnt.com/t5/airOS-Software-Configuration/How-can-i-see-signal-from-command-line/td-p/353927
        #https://community.ubnt.com/t5/airOS-Software-Configuration/How-can-I-remotely-read-RSSI/td-p/249199
        res = session.get('http://192.168.1.20/status.cgi')
        #signal = res.content.split('\n')[21][11:14]
        rssi = res.content.split('\n')[21][24:26]
        #print "Signal={}, rssi={}\n".format(signal, rssi)
        return int(rssi)

if __name__ == '__main__':
    print "Running ServoControl as Main.."
    configServos()
    findMaxRSSI()
