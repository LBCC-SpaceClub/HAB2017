import math
from math import radians, degrees, sin, cos, atan2, tan
import serial
import time
import serial
import serial.tools.list_ports
from threading import Thread
import geomag

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

		# Targetting solution fields:
		self.targetAziDeg = 0
		self.targetEleDeg = 0
		self.targetDistanceM = 0
		self.magneticVariation = 0

		# Connect to the arduino
		self.arduino = Arduino(parent)

		# Connect to the tracking servos
		self.servos = Servo(parent)


	def run(self):
		while self.running:
			# Simplify by using some temp variables
			stationConnected = self.parent.ids.station_connect.disabled
			stationManualButton = self.parent.ids.station_switchmanual.active
			payloadConnected = self.parent.ids.payload_connect.disabled
			payloadManualButton = self.parent.ids.payload_switchmanual.active
			motorManualButton = self.parent.ids.motor_switchmanual.active
			#if the motor manual button is off, run like normal
			if(not motorManualButton):
				# Unless Manual button enabled, check for new arduino gps
				if self.arduino.connected and not stationManualButton:
					self.update()
					self.updateMain()

				# Update the gui compasses if both gps positions available
				hasStationGps = (stationConnected or stationManualButton)
				hasPayloadGps = (payloadConnected or payloadManualButton)
				if (hasStationGps and hasPayloadGps):
					self.updateTargetSolution()
					self.updateGuiCompass()

			#move based on calculations or manually
			# Only run servos if motors are enabled
			if self.servos.connected:
				self.moveMotors()

			# No point updating faster than new data becomes available
			if(not motorManualButton):
				time.sleep(1)

	def moveMotors(self):
		if self.parent.ids.motor_switchstop.active:
			'''
			self.servos.stopMoving(self.servos.aziChannel) #stop the movement
			self.servos.stopMoving(self.servos.eleChannel)
			'''
			#command the servos to stop moving
			self.servos.stopMoving()
			return		# Don't move if the safety lockout is engaged!
		if self.servos.connected:
			pan = float(self.parent.x_value)
			tilt = float(self.parent.y_value)
			self.servos.pointTo(pan, tilt)
			#don't print if manually pointing to avoid spam
			if(not self.parent.ids.motor_switchmanual.active):
				print("Moving servos to: ", pan, tilt)



	def update(self):
		while self.arduino.hasLines():
			try:
				line = self.arduino.getLine()
				if line.startswith('[IMU]'):
					self.parseIMU(line[5:])
				elif line.startswith('[GPS]'):
					self.parseGPS(line[5:])
				elif line.startswith('Time: '):
					self.parseTime(line)
			except Exception as e:
				print("Exception on line from arduino: ", e)


	def updateMain(self):
		self.parent.ids.station_lat.text = str(self.latDeg)
		self.parent.ids.station_long.text = str(self.lonDeg)
		self.parent.ids.station_alt.text = str(self.altMeters)
		self.parent.ids.station_trueHeading.text = str(self.imuX)
		self.parent.ids.station_time.text = str(self.gpsTime)


	def updateGuiCompass(self):
		if self.parent.ids.station_switchmanual.active:
			self.updateTargetSolution()

		self.parent.x_value = self.targetAziDeg
		self.parent.y_value = self.targetEleDeg


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
		print("Parsing GPS line: ", line)
		line = line.split(',')
		self.latDeg = float(line[0])
		self.lonDeg = float(line[1])
		self.altMeters = float(line[2])



	def parseTime(self, line):
		line = line.split()
		self.gpsTime = line[1]
		self.gpsDate = line[3]


	def updateTargetSolution(self):
		try:
			pLat = float(self.parent.ids.payload_lat.text)
			pLon = float(self.parent.ids.payload_long.text)
			pAlt = float(self.parent.ids.payload_alt.text)

			if self.parent.ids.station_switchmanual.active:
				# Use manually entered GPS values
				tLat = float(self.parent.ids.station_lat.text)
				tLon = float(self.parent.ids.station_long.text)
				tAlt = float(self.parent.ids.station_alt.text)
			else:
				# Use latest values (should be the same?)
				tLat = float(self.latDeg)
				tLon = float(self.lonDeg)
				tAlt = float(self.altMeters)

			self.targetDistanceM = self.getTargetDistance(
				tLat, tLon, pLat, pLon
			)
			self.targetEleDeg = self.getEleDegrees(
				pAlt, tAlt, self.targetDistanceM
			)
			self.targetAziDeg = self.getAzDegrees(
				tLat, tLon, pLat, pLon
			)
		except ValueError as e:
			print("updateTargetSolution() error: ", e)
			# defaults
			self.targetDistanceM = 0
			self.targetEleDeg = 0
			self.targetAziDeg = 0


	def getEleDegrees(self, payloadAlt, stationAlt, distance):
		''' Returns degrees from perpendicular to Earth surface, to payload '''
		try:
			deltaAlt = payloadAlt - stationAlt
			return float(math.degrees(math.atan2(deltaAlt, distance)))
		except (ValueError, TypeError) as e:
			print("Get Elevation Error: ", e)
			return 0


	def getTargetDistance(self, trackerLat, trackerLon, remoteLat, remoteLon):
		''' Returns distance over a sphere, between two points '''
		# haversine formula, see: http://www.movable-type.co.uk/scripts/latlong.html
		try:
			R = 6371000									# radius of earth in meters
			dLat = math.radians(remoteLat-trackerLat)	# delta latitude in radians
			dLon = math.radians(remoteLon-trackerLon)	# delta longitude in radians
			a = math.sin(dLat/2)*math.sin(dLat/2)+math.cos(math.radians(trackerLat))*math.cos(math.radians(remoteLat))*math.sin(dLon/2)*math.sin(dLon/2)
			c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a))
			d = R*c
			return int(d)
			# return d*3280.839895 # multiply distance in Km by 3280 for feet
		except (ValueError, TypeError) as e:
			print("Get Distance Error: ", e)
			return 0


	def getAzDegrees(self, trackerLat, trackerLon, remoteLat, remoteLon):
		''' Returns degrees from true North, to payload '''
		try:
			dLat = math.radians(remoteLat-trackerLat)		# delta latitude in radians
			dLon = math.radians(remoteLon-trackerLon)		# delta longitude in radians

			y = math.sin(dLon)*math.cos(math.radians(remoteLat))
			x = math.cos(math.radians(trackerLat))*math.sin(math.radians(remoteLat))-math.sin(math.radians(trackerLat))*math.cos(math.radians(remoteLat))*math.cos(dLat)
			tempBearing = math.degrees(math.atan2(y,x))		# returns the bearing from true north
			tempBearing = tempBearing % 360		#keeps it within 0-360
			return float(tempBearing)
		except ValueError as e:
			print ("Get Azimuth Error: ", e)
			return 0



##################################################
###
###     Servo Methods
###
##################################################
class Servo(object):
	''' Methods to move the tracking servos '''

	def __init__(self, parent):
		# Arduino (USB) fields
		self.connected = False
		self.parent = parent
		self.servoBaud = 9600
		self.servoTimeout = 1
		#Maestro commands
		self.moveCommand = 0xFF
		self.moveCommandCompact = 0x84
		self.speedCommand = 0x87
		self.accCommand = 0x89
		self.checkMovingCommand = 0x93
		#make sure to copy/paste and upload stopMovingScript.txt to the maestro controller through Pololu Maestro Controll Center
		self.stopMovingCommand = [0xA7, 0x00]
		self.getPosition = 0x90
		self.PololuCmd = chr(0xAA) + chr(0x0C)
		#Servo ranges
		self.minRange = [0, 0, 0, 0]
		self.maxRange = [0, 0, 255, 134]
		# Pan and tilt servos on different channels
		self.aziChannel = 2
		self.eleChannel = 3
		# Connect to maestro servo controller
		self.servoCOM = self.findComPort()
		self.connect()
		self.configServos()


	def __del__(self):
		if self.connected:
			self.usb.close()


	def connect(self):
		if self.servoCOM:
			self.usb = serial.Serial(
				self.servoCOM,
				baudrate = self.servoBaud,
				timeout = self.servoTimeout
			)
			self.connected = True
			self.parent.updateConsole("\tConnected to maestro on port "+self.servoCOM)
			print("connected to maestro board")

	def findComPort(self):
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			if 'Pololu Micro Maestro 6-Servo Controller Command Port' in p[1]:
				print("Maestro Using: ", p[0])
				return p[0]
		self.parent.updateConsole(" **WARNING** could not find a maestro port")
		return None


	def configServos(self):
		#Set acceleration rate for both servos
		print("Configuring servos..")
		self.setSpeed(self.aziChannel, 4)
		self.setSpeed(self.eleChannel, 4)


	def degToServo(self, deg):
		"""
		''' Converts 0-360 degrees to 3000-9000 microsteps '''
		# remove any extra 360s and ensure we're positive degrees
		deg = abs(deg % 360)

		# steps = (degrees-degMin) * (stepMax-stepMin) / degMax + stepMin
		return int(deg * 6000.0 / 360.0 + 3000)
		"""
		''' Converts 0-360 degrees to 0-255 servo positions '''
		# remove any extra 360s
		deg = abs(deg % 360)

		#convert if it is on the right hemisphere
		if deg < 180:
			#subtract from the 0 degree position
			position = round(127 - deg * (255.0 / 360.0))

		#convert if it is on the left hemisphere
		else:
			#get the degree position going left from center
			deg = 360 - deg
			# add from the 0 degree position
			position = round(127 + deg * (255.0 / 360.0))
		return int(position)




	def setTarget(self, chan, deg):
		# Convert from degrees into servo
		target = self.degToServo(deg)

		# Ensure we don't move past our physical range for this servo
		if self.minRange[chan] > 0 and target < self.minRange[chan]:
			target = self.minRange[chan]
		if self.maxRange[chan] > 0 and target > self.maxRange[chan]:
			target = self.maxRange[chan]

		#print("Channel ", chan, " target in Servo is: ", target)
		#command, channel, value to servo
		cmd = [self.moveCommand, chan, target]
		self.usb.write(cmd)

	# Set speed of channel
	# Speed is measured as 0.25microseconds/10milliseconds
	# For the standard 1ms pulse width change to move a servo between extremes, a speed
	# of 1 will take 1 minute, and a speed of 60 would take 1 second.
	# Speed of 0 is unrestricted.
	def setSpeed(self, chan, speed):
		lsb = speed & 0x7f #7 bits for least significant byte
		msb = (speed >> 7) & 0x7f #shift 7 and take next 7 bits for msb
		# Send Pololu intro, device number, command, channel, speed lsb, speed msb
		cmd = [self.speedCommand, chan, lsb, msb]
		self.usb.write(cmd)

	# Set acceleration of channel
	# This provide soft starts and finishes when servo moves to target position.
	# Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
	# A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
	def setAccel(self, chan, accel):
		lsb = accel & 0x7f #7 bits for least significant byte
		msb = (accel >> 7) & 0x7f #shift 7 and take next 7 bits for msb
		# Send Pololu intro, device number, command, channel, accel lsb, accel msb
		cmd = [self.accCommand, chan, lsb, msb]
		self.usb.write(cmd)

	def pointTo(self, pan, tilt):
		self.setTarget(self.aziChannel, pan)	# pan
		self.setTarget(self.eleChannel, tilt)	# tilt

	def checkMoving(self):
		#send command to check if the servos are still moving
		self.usb.write(self.checkMovingCommand)
		#read the controller's response (still moving is 1, not moving is 0)
		movingStatus = self.usb.read()
		#return boolean based on response
		if(movingStatus == 1):
			return True
		else:
			return False

	def getServoPosition(self, chan):
		#command to get the current position of the servo
		cmd = [self.getPosition, chan]
		self.usb.write(cmd)
		#get the response
		lsb = self.usb.read()
		msb = self.usb.read()
		return {'lsb': lsb, 'msb': msb}
		#print("lsb: ", self.usb.read())
		#print("msb: ", self.usb.read())

	def stopMoving(self):
		#make sure to copy/paste and upload stopMovingScript.txt to the maestro controller through Pololu Maestro Controll Center
		#command to run stop moving script on maestro controller
		self.usb.write(self.stopMovingCommand)

##################################################
###
###     Arduino Methods
###
##################################################
class Arduino(object):
	''' Methods to connect to the USB arduino '''
	def __init__(self, parent):
		# Arduino (USB) fields
		self.connected = False # default to not connected
		self.parent = parent
		self.arduinoBaud = 115200 # might be too high, I'm seeing some garbage
		self.arduinoTimeout = 1
		self.arduinoCOM = self.findComPort()
		self.connect()
		if not self.connected:
			raise IOError('Could not connect to arduino')


	def __del__(self):
		if self.connected:
			self.usb.close()


	def connect(self):
		self.usb = serial.Serial(
			self.arduinoCOM,
			baudrate = self.arduinoBaud,
			timeout = self.arduinoTimeout
		)
		self.usb.flush()
		self.connected = True
		self.parent.updateConsole("\tConnected to servo arduino on port "+self.arduinoCOM)


	def findComPort(self):
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			if 'Arduino' in p[1] or 'ttyACM' in p[1]:
				print("Arduino Using: ", p[0])
				return p[0]
		raise IOError('Could not find arduino port')


	def getLine(self):
		return self.usb.readline().decode("utf-8")


	def hasLines(self):
		return self.usb.inWaiting()
