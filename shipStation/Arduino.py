#!/usr/bin/env python
import serial

class Arduino(object):
    ''' Methods to get information from the arduino (gps and IMU) '''
    _arduinoCOM = ""
    _arduinoBaud = 115200
    _arduinoTimeout = 5
    _arduinoAttached = False
    ser = None

    def __init__(self):
        self._arduinoCOM = findComPort()
        self.ser = serial.Serial(
            str(self._arduinoCOM),
            baudrate = self._arduinoBaud,
            timeout = self._arduinoTimeout
        )

    def findComPort(self):
        # find the actual com port
        return 'COM5'

    def calibrateIMU(self):
        calibration = 0
        calibrationGoal = 8
        temp_arduino = None
        while (calibration < calibrationGoal):
            ser.flushInput()
            time.sleep(0.05)
            while(temp_arduino[0] != '~'):
                temp_arduino = ser.readline()
                print temp_arduino
                temp_arduino = temp_arduino.split(',')
            try:
                calibration = int(temp_arduino[8])+int(temp_arduino[7])+int(temp_arduino[6])
            finally:
                print "Calibration: ",calibration, " // Goal of ", int(calibrationGoal)
        ser.flushInput()

    def update(self):


def arduinoGround():
    #Gather data from arduino mounted on ground station
    global groundAlt,groundLat,groundLon,centerBear,antennaBear,calibrationGoal
    if arduinoAttached:
            s2 =serial.Serial(str(arduinoCOM), baudrate = arduinoBaud, timeout = arduinoTimeout)
            time.sleep(5)
            temp_arduino = "0";
            calibration = 0;
            x = 64;
            y = 70;
            bad = 0
            while (calibration < int(calibrationGoal)):
                temp_arduino = "0"
                s2.flushInput()
                time.sleep(0.05)
                while(temp_arduino[0] != '~'):
                    temp_arduino = s2.readline()
                    print temp_arduino
                    temp_arduino = temp_arduino.split(',')
                try:
                    #+int(temp_arduino[5])
                    calibration = int(temp_arduino[8])+int(temp_arduino[7])+int(temp_arduino[6])
                finally:
                    print "Calibration: ",calibration, " // Goal of ", int(calibrationGoal)
                    if (x > 191):
                        x = 64;
                        time.sleep(3)
                    else:
                        x += 10;
                    if (y < 70):
                        y = 127;
                    else:
                        y -= 3;
                    movePanServo(x)
                    moveTiltServo(y)
            moveToCenterPos()
            time.sleep(10)
            temp_arduino = "0"
            s2.flushInput()

            while(temp_arduino[0] != '~'):
                temp_arduino = s2.readline();
                print temp_arduino
                temp_arduino = temp_arduino.split(',');
            print "Requesting Latitude"
            tempLat = temp_arduino[1]
            print tempLat
            print "Requesting Longitude"
            tempLon = temp_arduino[2]
            print tempLon
            print "Requesting Altitude"
            tempAlt = temp_arduino[3]
            tempoffsetDegrees = "0.00"
            print tempAlt
            print "Requesting Orientation"
            tempoffsetDegrees = temp_arduino[4]
            print tempoffsetDegrees
            tempLat = tempLat.split(".")
            groundLat = float(tempLat[0])+float(tempLat[1])/10000000
            tempLon = tempLon.split(".")
            groundLon = float(tempLon[0])-float(tempLon[1])/10000000
            tempAlt = tempAlt.split(".")
            groundAlt = int(tempAlt[0])
            #tempoffsetDegrees = tempoffsetDegrees.split(".")
            centerBear = float(tempoffsetDegrees)
            declination = float(geomag.declination(dlat = groundLat,dlon = groundLon, h = groundAlt))
            s2.close()
    else:
        print "Error: Arduino set in Settings as not connected"
    print "Local Latitude: \t",groundLat
    print "Local Longitude:\t",groundLon
    print "Local Altitude: \t",groundAlt
    print "Offset Degrees: \t",centerBear
    print "Declination:    \t",declination
    centerBear = (centerBear+declination)
    if centerBear > 360:
        centerBear = centerBear - 360
    elif centerBear < 0:
        centerBear = centerBear + 360
    print "Offset + Dec:   \t",(centerBear)
    print "-------------------------------------------------------"
    antennaBear = (centerBear)
