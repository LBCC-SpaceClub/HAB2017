#!/usr/bin/env python
import serial
import serial.tools.list_ports

class Arduino:
    ''' Methods to get information from the arduino (gps and IMU) '''
    def __init__(self):
        self.arduinoCOM = self.findComPort()
        self.usb = serial.Serial(
            self.arduinoCOM,
            baudrate = self.arduinoBaud,
            timeout = self.arduinoTimeout
        )
        self.arduinoCOM = None
        self.arduinoBaud = 115200
        self.arduinoTimeout = 5
        # GPS fields
        self.gpsTime = None
        self.gpsDate = None
        self.latDeg = None
        self.lonDeg = None
        self.altMeters = None
        self.ser = None
        # IMU fields
        self.imuX = None
        self.imuY = None
        self.imuZ = None

    def __del__(self):
        ''' Cleans up when this object is destroyed '''
        if self.usb:
            self.setLog("Closing port.")
            self.usb.close()

    def findComPort(self):
        # find the actual com port of the arduino (windows only?)
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'Arduino' in p[1]:
                self.setLog("Found arduino on ", self.p[0])
                return p[0]
        self.usb = None
        raise IOError("ERROR: Could not find an attached arduino.")

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
                    self.setLog("Error reading line: "+line)
            except ValueError:
                self.setLog("Error parsing "+line)
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
        self.log= self.log+" "+txt

    
    def getLog(self):
        if(self.log != ""):
            return self.log
        else:
            return ""
    
    def clearLog(self):
        self.log = ""

if __name__ == "__main__":
    print ("Arduino is not a standalone Python program.")
