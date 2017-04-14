#!/usr/bin/env python
import serial
import serial.tools.list_ports

class Arduino:
    ''' Methods to get information from the arduino (gps and IMU) '''
    _arduinoCOM = None
    _arduinoBaud = 115200
    _arduinoTimeout = 5
    _arduinoAttached = False
    # GPS fields
    gpsTime = None
    gpsDate = None
    latDeg = None
    lonDeg = None
    altMeters = None
    ser = None
    # IMU fields
    imuX = None
    imuY = None
    imuZ = None


    def __init__(self):
        self._arduinoCOM = self.findComPort()
        if self._arduinoAttached:
            print "Found arduino on ", self._arduinoCOM
        else:
            print "Could not find an attached arduino."
            return None
        self._ser = serial.Serial(
            self._arduinoCOM,
            baudrate = self._arduinoBaud,
            timeout = self._arduinoTimeout
        )

    def __del__(self):
        ''' Cleans up when this object is destroyed '''
        print "Closing port."
        self._ser.close()

    def findComPort(self):
        # find the actual com port of the arduino (windows only?)
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'Arduino' in p[1]:
                _arduinoAttached = True
                return p[0]
        return None

    def calibrateIMU(self):
        ''' Poorly named as it doesn't force the arduino to DO anything '''
        calibration = 0
        calibrationGoal = 8
        temp_arduino = None
        while (calibration < calibrationGoal):
            _ser.flushInput()
            time.sleep(0.05)
            while(temp_arduino[0] != '~'):
                temp_arduino = _ser.readline()
                print temp_arduino
                temp_arduino = temp_arduino.split(',')
            try:
                calibration = int(temp_arduino[8])+int(temp_arduino[7])+int(temp_arduino[6])
            finally:
                print "Calibration: ",calibration, " // Goal of ", int(calibrationGoal)
        _ser.flushInput()

    def update(self):
        ''' Read in new data from the arduino '''
        while(self._ser.in_waiting):
            line = self._ser.readline()
            if line[:5] == '[IMU]':
                self.updateIMU(line[5:])
            elif line[:5] == '[GPS]':
                self.updateGPS(line[5:])
            else:
                print "Error reading line: ", line
        print "Most up-to-date Lat: {}, Long: {}, Alt (m): {}".format(
            self.latDeg,
            self.lonDeg,
            self.altMeters
        )
        print "Heading: ", imuX

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
