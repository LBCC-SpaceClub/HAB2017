import math
from math import radians, degrees, sin, cos, atan2, tan
import serial
import requests
import time


class StepperControl():

	def __init__(self):
		# General class fields
		self.log = ""
		self.run = True

		# Arduino (USB) fields
		self.arduinoBaud = 115200
		self.arduinoTimeout = 5
		self.arduinoCOM = None

		# Servo (serial) fields
		self.panOffset = 0     # + right, - left
		self.tiltOffset = 0    # + raise, - lower
		self.moveCommand = 0xFF
		self.minPan = 0
		self.maxPan = 255
		self.minTilt = 70
		self.maxTilt = 123
		self.panChannel = 1
		self.tiltChannel = 0

		# Connect to the arduino and start a thread to keep this class updated
		self.arduino = self.connectToArduino()
		if self.arduino:
			self.setLog(" something ")
		else:
			self.setLog(" nothing ")

	def __del__(self):
		if self.arduino:
			self.setLog(" **STOP** closing arduino port")
			self.arduino.close()



	##################################################
	###
	###     Connection Methods
	###
	##################################################
	def connectToArduino(self):
		try:
			self.arduinoCOM = self.findComPort()
			self.arduino = serial.Serial(
				self.arduinoCOM,
				baudrate = self.arduinoBaud,
				timeout = self.arduinoTimeout
			)
			return not self.arduino
		except:
			return False


	def findComPort(self):
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			if 'Arduino' in p[1]:
				self.setLog(" **UPDATE** found Arduino on ", self.p[0])
				self.connected = True
				return p[0]
			else:
				self.setLog(" **ERROR** could not find an attached Arduino")



	##################################################
	###
	###     Calibration Methods
	###
	##################################################
	def configServos(self, usb):
		self.setLog(" **UPDATE** configuring servos...")
		#Set acceleration rate for both servos
		accelCommand = 0x89
		tiltAccel = 1
		panAccel = 1
		setAccel = [accelCommand,self.tiltChannel,tiltAccel,0]
		usb.write(setAccel)
		setAccel = [accelCommand,self.panChannel,panAccel,0]
		usb.write(setAccel)
		#Set rotation rate for both servos
		speedCommand = 0x87
		tiltSpeed = 1
		panSpeed = 3
		setSpeed = [speedCommand,self.tiltChannel,tiltSpeed,0]
		usb.write(setSpeed)
		setSpeed = [speedCommand,self.panChannel,panSpeed,0]
		usb.write(setSpeed)



	##################################################
	###
	###     Update Values Methods
	###
	##################################################
	def update(self):
		while self.arduino.inWaiting():
			line = self.arduino.readline()
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
	###     Move Servos Methods
	###
	##################################################
	def moveToCenterPos(self, arduino):
		moveTiltServo(127, arduino)
		movePanServo(127, arduino)


	def moveTiltServo(self, degrees, arduino):
		degInServo = 254.0 / 360 * degrees
		if(degInServo < self.minTilt):
		    degInServo = self.minTilt
		elif(degInServo > self.maxTilt):
		    degInServo = self.maxTilt
		degInServo = int(round(degInServo))
		moveTilt = [self.moveCommand, self.tiltChannel, chr(degInServo)]
		arduino.write(degInServo)


	def movePanServo(self, position, arduino):
		movePan = [moveCommand,panChannel,chr(255-position)]
		arduino.write(movePan)


	def degToServo(deg):
		if deg >= 360:
			deg = d % 360
		if deg < 180:
			val = int(round(127 - deg*(255.0/360.0)))
		else:
			deg = 360 - d
			val = int(round(127 + deg*(255.0/360.0)))
		return val



	##################################################
	###
	###     Logging Methods
	###
	##################################################
	def setLog(self, txt):
		self.log= self.log+""+txt


	def getLog(self):
		if(self.log != ""):
			return self.log
		else:
			return ""

	def clearLog(self):
		self.log = ""
