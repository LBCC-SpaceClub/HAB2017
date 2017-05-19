#!/usr/bin/env python
import serial
import serial.tools.list_ports
from threading import Thread
import time

class ArduinoThread(Thread):
    ''' Methods to get information from the arduino (gps and IMU) '''
    def __init__(self):
        Thread.__init__(self)
        self.stop = True
        self.log = ""
        self.usb = None
        self.connected = False
        self.arduinoCOM = None
        self.arduinoBaud = 115200
        self.arduinoTimeout = 5
        # GPS fields
        self.gpsTime = ""
        self.gpsDate = ""
        self.latDeg = ""
        self.lonDeg = ""
        self.altMeters = ""
        self.ser = None
        # IMU fields
        self.imuX = None
        self.imuY = None
        self.imuZ = None


    def run(self):
        if(self.connectToArduino()):
            self.setLog(" **SUCCESS** arduino connected, parsing data")
            self.connected = True
            
            while(self.stop):
                self.update()
        else:
            self.setLog(" **ERROR** failed to connect to arudino usb")
            self.connected = False


    def __del__(self):
        ''' Cleans up when this object is destroyed '''
        if self.usb:
            self.setLog(" **STOP** closing port")
            self.usb.close()


    def connectToArduino(self):
        try:
            self.arduinoCOM = self.findComPort()
            self.usb = serial.Serial(
                self.arduinoCOM,
                baudrate = self.arduinoBaud,
                timeout = self.arduinoTimeout
            )
            if not self.usb:
                return True
            else:
                return False
        except:
            return False


    def findComPort(self):
        # find the actual com port of the arduino (windows only?)
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'Arduino' in p[1]:
                self.setLog(" **UPDATE** found arduino on ", self.p[0])
                self.connected = True
                return p[0]
            else:
                self.setLog(" **ERROR** could not find an attached arduino") 
    

    def calibrateIMU(self):
        ''' Poorly named as it doesn't force the arduino to DO anything '''
        calibration = 0
        calibrationGoal = 8
        temp_arduino = None
        while (calibration < calibrationGoal):
            usb.flushInput()
            time.sleep(0.05)
            while(temp_arduino[0] != '~'):
                temp_arduino = usb.readline()
                self.setLog(temp_arduino)
                temp_arduino = temp_arduino.split(',')
            try:
                calibration = int(temp_arduino[8])+int(temp_arduino[7])+int(temp_arduino[6])
            finally:
                self.setLog("Calibration: "+calibration+" // Goal of ",+str(int(calibrationGoal)))
        usb.flushInput()


    def update(self):
        ''' Read in new data from the arduino '''
        while self.usb.inWaiting():
            line = self.usb.readline()
            try:
                if line[:5] == '[IMU]':
                    self.updateIMU(line[5:])
                elif line[:5] == '[GPS]':
                    self.updateGPS(line[5:])
                else:
                    self.setLog(" **ERROR** reading line: "+line)
            except ValueError:
                self.setLog(" **ERROR** parsing "+line)
        '''
        print "Most up-to-date Lat: {}, Long: {}, Alt (m): {}".format(
            self.latDeg,
            self.lonDeg,
            self.altMeters
        )
        print "Heading: ", self.imuX
        '''


    def updateIMU(self, line):
        line = line.split(',')
        self.imuX = float(line[0])
        self.imuY = float(line[1])
        self.imuZ = float(line[2])
        self.imuSys = int(line[3])
        self.imuAcc = int(line[4])
        self.imuGyro = int(line[5])
        self.imuMag = int(line[6])


    def updateGPS(self, line):
        line = line.split(',')
        self.latDeg = float(line[0])
        self.lonDeg = float(line[1])
        self.altMeters = float(line[2])


    def setLog(self, txt):
        self.log= self.log+""+txt


    def getLog(self):
        if(self.log != ""):
            return self.log
        else:
            return ""


    def clearLog(self):
        self.log = ""


if __name__ == "__main__":
    print ("Arduino is not a standalone Python program.")
