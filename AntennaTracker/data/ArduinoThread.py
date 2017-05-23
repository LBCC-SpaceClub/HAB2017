import serial
import serial.tools.list_ports
import time
from threading import Thread
from data.ServoControl import *


class ArduinoThread(Thread):

    arduinoBaud = 115200
    arduinoTimeout = 5
    arduinoCOM = None
    servo_control = ServoControl()


    def __init__(self):
        Thread.__init__(self)
        self.stop = True
        self.log = ""
        self.usb = None
        self.connected = False
        self.gpsTime = ""
        self.gpsDate = ""
        self.latDeg = ""
        self.lonDeg = ""
        self.altMeters = ""
        self.ser = None
        self.imuX = None
        self.imuY = None
        self.imuZ = None


    def run(self):
        if(self.connectToArduino()):
            self.setLog(" **SUCCESS** Arduino USB connected, parsing data")
            self.connected = True

            while(self.stop):
                self.updateData()
        else:
            self.setLog(" **ERROR** Failed to connect to Arduino USB")
            self.connected = False


    def __del__(self):
        if self.usb:
            self.setLog(" **STOP** Closing port")
            self.usb.close()



    ##################################################
    ###
    ###     Connection Methods
    ###
    ##################################################
    def connectToArduino(self):
        try:
            self.arduinoCOM = self.findComPort()
            self.usb = serial.Serial(
                self.arduinoCOM,
                baudrate = self.arduinoBaud,
                timeout = self.arduinoTimeout
            )
            return not self.usb
        except:
            return False


    def findComPort(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'Arduino' in p[1]:
                self.setLog(" **UPDATE** Found Arduino on ", self.p[0])
                self.connected = True
                return p[0]
            else:
                self.setLog(" **ERROR** Could not find an attached Arduino")



    ##################################################
    ###
    ###     Calibration Methods
    ###
    ##################################################
    def runCalibrateIMU(self):
        self.thread = Thread(target=calibrateIMU)
        self.thread.start()
        self.thread.join()


    def calibrateIMU(self):
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



    ##################################################
    ###
    ###     Servo Control
    ###
    ##################################################
    def servoMoveToCenter(self):
        if self.usb:
            self.servo_control.moveToCenterPos(self.usb)


    def servoMoveTilt(self, degrees):
        if self.usb:
            self.servo_control.moveTiltServo(degrees ,self.usb)


    def servoMovePan(self, pos):
        if self.usb:
            self.servo_control.movePanServo(pos ,self.usb)



    ##################################################
    ###
    ###     Update Values Methods
    ###
    ##################################################
    def updateData(self):
        while self.usb.inWaiting():
            line = self.usb.readline()
            try:
                if line[:5] == '[IMU]':
                    self.updateIMU(line[5:])
                elif line[:5] == '[GPS]':
                    self.updateGPS(line[5:])
                else:
                    self.setLog(" **ERROR** Reading line: "+line)
            except ValueError:
                self.setLog(" **ERROR** Parsing "+line)


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



    ##################################################
    ###
    ###     Logging Methods
    ###
    ##################################################
    def setLog(self, txt):
        self.log = self.log+""+txt


    def getLog(self):
        if(self.log != ""):
            return self.log
        else:
            return ""


    def clearLog(self):
        self.log = ""
