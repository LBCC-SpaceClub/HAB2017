import math
from math import radians, degrees, sin, cos, atan2, tan
import serial
import requests
import time

import serial
import serial.tools.list_ports
from threading import Thread

class ServoControl(Thread):

	def __init__(self, parent):
		Thread.__init__(self)
		# General class fields
		self.parent = parent
		self.connected = False
		self.running = True
		self.daemon = True		#stops thread on app exit, important

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
		self.arduino = Arduino(parent)
		if not self.arduino.connect():
			raise IOError('Could not connect to arduino')

		# Connect to the tracking servos
		self.servos = Servo(parent)
		if not self.servos.connected:
			self.parent.updateConsole(" **WARNING** could not connect to maestro")


	def run(self):
		while self.running:
			self.update()
			self.updateMain()
			time.sleep(1)


	def update(self):
		while self.arduino.hasLines():
			try:
				line = self.arduino.getLine()
				if line[:5] == '[IMU]':
					self.parseIMU(line[5:])
				elif line[:5] == '[GPS]':
					self.parseGPS(line[5:])
				elif line.startswith('Time: '):
					self.parseTime(line)
			except:
				print("Exception on: ", line)
				# self.parent.updateConsole(" **ERROR** Parsing line from arduino")

	def updateMain(self):
		self.parent.ids.station_lat.text = str(self.latDeg)
		self.parent.ids.station_long.text = str(self.lonDeg)
		self.parent.ids.station_alt.text = str(self.altMeters)
		self.parent.ids.station_trueHeading.text = str(self.imuX)
		self.parent.ids.station_time.text = str(self.gpsTime)


	def parseIMU(self, line):
		line = line.split(',')
		self.imuX = float(line[0])
		self.imuY = float(line[1])
		self.imuZ = float(line[2])
		self.imuSys = int(line[3])
		self.imuAcc = int(line[4])
		self.imuGyro = int(line[5])
		self.imuMag = int(line[6])


	def parseGPS(self, line):
		line = line.split(',')
		self.latDeg = float(line[0])
		self.lonDeg = float(line[1])
		self.altMeters = float(line[2])


	def parseTime(self, line):
		line = line.split()
		self.gpsTime = line[1]
		self.gpsDate = line[3]


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
###     Servo Methods
###
##################################################
class Servo(object):
	''' Methods to move the tracking servos '''

	def __init__(self, parent):
		# Arduino (USB) fields
		self.parent = parent
		self.connected = False
		self.servoBaud = 9600
		self.servoTimeout = 1
		self.servoCOM = self.findComPort()
		# Servo ranges
		self.moveCommand = 0xFF
		self.minPan = 0
		self.maxPan = 255
		self.minTilt = 70
		self.maxTilt = 123
		# Pan and tilt servos on different channels
		self.panChannel = 1
		self.tiltChannel = 0
		self.connect()
		self.configServos()


	def __del__(self):
		if self.connected:
			self.usb.close()


	def connect(self):
		try:
			self.usb = serial.Serial(
				self.servoCOM,
				baudrate = self.servoBaud,
				timeout = self.servoTimeout
			)
			self.connected = True
		except portNotOpenError:
			self.parent.updateConsole(" **ERROR** could not find a maestro port")
		return self.connected


	def findComPort(self):
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			if 'Pololu Micro Maestro 6-Servo Controller' in p[1]:
				return p[0]


	def configServos(self):
		self.parent.updateConsole(" **UPDATE** configuring servos...")
		#Set acceleration rate for both servos
		accelCommand = 0x89
		tiltAccel = 1
		panAccel = 1
		setAccel = [accelCommand, self.tiltChannel, tiltAccel, 0]
		self.usb.write(setAccel)
		setAccel = [accelCommand, self.panChannel, panAccel, 0]
		self.usb.write(setAccel)
		#Set rotation rate for both servos
		speedCommand = 0x87
		tiltSpeed = 1
		panSpeed = 3
		setSpeed = [speedCommand, self.tiltChannel, tiltSpeed, 0]
		self.usb.write(setSpeed)
		setSpeed = [speedCommand, self.panChannel, panSpeed, 0]
		self.usb.write(setSpeed)


		def moveToCenter():
			moveTilt(127)
			movePan(127)


		def moveAz(position):
			if(position < 70):          #80 degrees upper limit
				moveTilt = [moveCommand,tiltChannel,chr(70)]
			elif(position > 123):       #5 degrees lower limit
				moveTilt = [moveCommand,tiltChannel,chr(123)]
			else:
				moveTilt = [moveCommand,tiltChannel,chr(position)]
			s.write(moveTilt)


		def moveEle(position):
			movePan = [moveCommand,panChannel,chr(255-position)]
			s.write(movePan)


##################################################
###
###     Arduino Methods
###
##################################################
class Arduino(object):
	''' Methods to connect to the USB arduino '''
	def __init__(self, parent):
		# Arduino (USB) fields
		self.arduinoBaud = 115200
		self.arduinoTimeout = 1
		self.arduinoCOM = None
		self.connected = False
		self.usb = None
		self.parent = parent


	def __del__(self):
		if self.connected:
			self.usb.close()


	def connect(self):
		try:
			self.arduinoCOM = self.findComPort()
			self.usb = serial.Serial(
				self.arduinoCOM,
				baudrate = self.arduinoBaud,
				timeout = self.arduinoTimeout
			)
			self.usb.flush()
			return True
		except:
			self.parent.updateConsole(" **ERROR** could not find arduino port")
			return False


	def findComPort(self):
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			if 'Arduino' in p[1] or 'ttyACM' in p[1]:
				return p[0]

		self.parent.updateConsole(" **ERROR** manually select servo arduino port")
		raise IOError('Could not connect to arduino')


	def getLine(self):
		return self.usb.readline().decode("utf-8")


	def hasLines(self):
		return self.usb.inWaiting()
