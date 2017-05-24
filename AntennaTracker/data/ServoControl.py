import math
from math import radians, degrees, sin, cos, atan2, tan
import serial
import requests
import time

import serial
import serial.tools.list_ports
from threading import Thread
from data.ServoControl import *


class ServoControl(Thread):

	def __init__(self, parent):
		Thread.__init__(self)
		# General class fields
		self.parent = parent
		self.connected = False
		self.running = True
		self.daemon = True		#stops thread on app exit, important

		# Arduino (USB) fields
		self.arduinoBaud = 115200
		self.arduinoTimeout = 5
		self.arduinoCOM = None

		# Servo (serial) fields
		self.panOffset = 0		# + right, - left
		self.tiltOffset = 0		# + raise, - lower
		self.moveCommand = 0xFF
		self.minPan = 0
		self.maxPan = 255
		self.minTilt = 70
		self.maxTilt = 123
		self.panChannel = 1
		self.tiltChannel = 0

		# Orientation fields
		self.imuX = None
		self.imuY = None
		self.imuZ = None
		self.imuSys = None
		self.imuAcc = None
		self.imuGyro = None
		self.imuMag = None
		self.latDeg = "Connecting.."
		self.lonDeg = "Connecting.."
		self.altMeters = None
		self.gpsDate = None
		self.gpsTime = None

		# Connect to the arduino and start a thread to keep this class updated
		self.connected = self.connectToArduino()
		self.arduino.flush()


	def __del__(self):
		if self.connected:
			self.parent.updateConsole(" **STOP** closing arduino port")
			self.arduino.close()


	def run(self):
		while self.running:
			self.update()
			self.updateMain()
			time.sleep(1)



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
			# time.sleep(2)
			return True
		except:
			self.parent.updateConsole(" **ERROR** could not find a servo arduino")
			return False


	def findComPort(self):
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			print(p)
			if 'Arduino' in p[1] or 'ttyACM' in p[1]:
				# self.parent.updateConsole(" **UPDATE** found Arduino on ", p[0])
				# self.parent.updateConsole(" **UPDATE** found Arduino on ", p[0])
				return p[0]

		self.parent.updateConsole(" **ERROR** could not find an attached Arduino")
		return None



	##################################################
	###
	###     Calibration Methods
	###
	##################################################
	def configServos(self, usb):
		self.parent.updateConsole(" **UPDATE** configuring servos...")
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
			line = self.arduino.readline().decode("utf-8")
			try:
				if line[:5] == '[IMU]':
					self.updateIMU(line[5:])
				elif line[:5] == '[GPS]':
					self.updateGPS(line[5:])
				else:
					print("Could not parse: ", line)
				# self.parent.updateConsole(" **ERROR** Reading line: "+line)
			except:
				print("Exception on: ", line)
				# self.parent.updateConsole(" **ERROR** Parsing line from arduino")

	def updateMain(self):
		self.parent.ids.station_lat.text = str(self.latDeg)
		self.parent.ids.station_long.text = str(self.lonDeg)
		self.parent.ids.station_alt.text = str(self.altMeters)
		self.parent.ids.station_trueHeading.text = str(self.imuX)
		self.parent.ids.station_time.text = str(self.gpsTime)


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
