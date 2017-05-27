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
		self.targetAzDeg = 0
		self.targetEleDeg = 0
		self.targetDistanceM = 0

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

			# Unless Manual button enabled, check for new arduino gps
			if not stationManualButton:
				self.update()
				self.updateMain()

			# Update the gui compasses if both gps positions available
			hasStationGps = (stationConnected or stationManualButton)
			hasPayloadGps = (payloadConnected or payloadManualButton)
			if (hasStationGps and hasPayloadGps):
				self.updateGuiCompass()
				# Only run servos if both gps positions AND motors are enabled
				if self.servos.connected:
					self.moveMotors()

			# No point updating faster than new data becomes available
			time.sleep(1)


	def moveMotors(self):
		if self.parent.ids.motor_switchstop.active:
			return		# Don't move if the safety lockout is engaged!
		if self.servos.connected:
			# print("Moving servos to ele = {:.3f}, az = {:.3f} degrees.".format(
			# 	self.targetEleDeg, self.targetAzDeg)
			# )
			self.servos.moveEle(float(self.targetEleDeg))
			self.servos.moveAz(float(self.targetAzDeg))


	def update(self):
		while self.arduino.hasLines():
			try:
				line = self.arduino.getLine()
				if line[:5] == '[IMU]':
					self.parseIMU(line[5:])
				elif line[:5] == '[GPS]':
					self.parseGPS(line[5:])
					self.updateTargetSolution()
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

		self.parent.x_value = self.targetAzDeg
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
				tLat = self.latDeg
				tLon = self.lonDeg
				tAlt = self.altMeters

			self.targetDistanceM = self.getTargetDistance(
				tLat, tLon, pLat, pLon
			)
			self.targetEleDeg = self.getEleDegrees(
				pAlt, tAlt, self.targetDistanceM
			)
			self.targetAzDeg = self.getAzDegrees(
				tLat, tLon, pLat, pLon
			)
		except ValueError as e:
			print(e)
			# defaults
			self.targetDistanceM = 0
			self.targetEleDeg = 0
			self.targetAzDeg = 0


	def getEleDegrees(self, payloadAlt, stationAlt, distance):
		''' Returns degrees from perpendicular to Earth surface, to payload '''
		try:
			deltaAlt = payloadAlt - stationAlt
			return float(math.degrees(math.atan2(deltaAlt, distance)))
		except ValueError:
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
		except ValueError:
			return 0


	def getAzDegrees(self, trackerLat, trackerLon, remoteLat, remoteLon):
		''' Returns degrees from true North, to payload '''
		try:
			dLat = math.radians(remoteLat-trackerLat)		# delta latitude in radians
			dLon = math.radians(remoteLon-trackerLon)		# delta longitude in radians

			y = math.sin(dLon)*math.cos(math.radians(remoteLat))
			x = math.cos(math.radians(trackerLat))*math.sin(math.radians(remoteLat))-math.sin(math.radians(trackerLat))*math.cos(math.radians(remoteLat))*math.cos(dLat)
			tempBearing = math.degrees(math.atan2(y,x))		# returns the bearing from true north
			if (tempBearing < 0):
				tempBearing = tempBearing + 360
			return float(tempBearing)
		except ValueError:
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
		# Servo ranges
		self.moveCommand = 0xFF
		self.minAz = 0
		self.maxAz = 255
		self.minEle = 70
		self.maxEle = 180
		# Pan and tilt servos on different channels
		self.panChannel = 1
		self.tiltChannel = 0
		# Connect to maestro servo controller
		self.servoCOM = self.findComPort()
		if self.servoCOM:
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
			self.usb.flush()
			self.connected = True
			self.parent.updateConsole("\tConnected to maestro on port "+self.servoCOM)


	def findComPort(self):
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			if 'Pololu Micro Maestro 6-Servo Controller' in p[1]:
				return p[0]
		self.parent.updateConsole(" **WARNING** could not find a maestro port")
		return None


	def configServos(self):
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


	def degToServo(self, deg):
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


	def moveToCenter(self):
		moveAz(127)
		moveEle(127)


	def moveAz(self, deg):
		if self.parent.ids.motor_switchstop.active:
			return		# Don't move if the safety lockout is engaged!

		position = self.degToServo(deg)
		if(position < self.minAz):
			print("Servo Warning: {} < minAz=={}.".format(position, self.minAz))
			cmd = [self.moveCommand, self.tiltChannel, self.minAz]
		elif(position > self.maxAz):
			print("Servo Warning: {} > maxAz=={}.".format(position, self.maxAz))
			cmd = [self.moveCommand, self.tiltChannel, self.maxAz]
		else:
			cmd = [self.moveCommand, self.tiltChannel, position]
		self.usb.write(cmd)


	def moveEle(self, deg):
		if self.parent.ids.motor_switchstop.active:
			return		# Don't move if the safety lockout is engaged!

		position = self.degToServo(deg)
		if(position < self.minEle):
			print("Servo Warning: {} < minEle=={}.".format(position, self.minEle))
			cmd = [self.moveCommand, self.panChannel, self.minEle]
		elif(position > self.maxEle):
			print("Servo Warning: {} > maxEle=={}.".format(position, self.maxEle))
			cmd = [self.moveCommand, self.panChannel, self.maxEle]
		else:
			cmd = [self.moveCommand, self.panChannel, position]
		self.usb.write(cmd)


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
				return p[0]
		raise IOError('Could not find arduino port')


	def getLine(self):
		return self.usb.readline().decode("utf-8")


	def hasLines(self):
		return self.usb.inWaiting()
