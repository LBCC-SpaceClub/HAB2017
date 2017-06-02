import math
from math import radians, degrees, sin, cos, atan2, tan
import serial
import time
import serial
import serial.tools.list_ports
from threading import Thread
import geomag

class StepperControl(Thread):

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
		print("Steppers created!")

	def run(self):
		while self.running:
			print("Steppers Running!")
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
				# Only run motors if both gps positions AND motors are enabled
				if self.arduino.connected:
					self.moveMotors()

			# No point updating faster than new data becomes available
			time.sleep(1)


	def moveMotors(self):
		if self.parent.ids.motor_switchstop.active:
			return		# Don't move if the safety lockout is engaged!
		if self.arduino.connected:
			# print("Moving motors to ele = {:.3f}, az = {:.3f} degrees.".format(
			# 	self.targetEleDeg, self.targetAzDeg)
			# )
			self.arduino.moveBoth(float(self.targetEleDeg), float(self.targetAzDeg))
			# self.arduino.moveEle(float(self.targetEleDeg))
			# self.arduino.moveAz(float(self.targetAzDeg))


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
		self.magDeclination = geomag.declination(
            dlat=self.latDeg, dlon=self.lonDeg, h=self.altMeters
        )


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
		self.prevAziSteps = 0
		self.prevEleSteps = 0
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
		self.parent.updateConsole("\tConnected to stepper arduino on port "+self.arduinoCOM)


	def sendPayloadGpsToArduino():
		curTime = time.strftime('%H%M%S')
		'$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A'
		if magDeclination > 0:
			magSuffix = 'E'
		else:
			magSuffix = 'W'
		# GPRMC does not include altitude.. add as custom field?
		nmea = "GPRMC,{now},A,{lat},{lon},,,{mag},{magSuffix}".format(
			now = curTime,
			lat = decDegToNMEA(self.parent.ids.payload_lat.text)
			lon = decDegToNMEA(self.parent.ids.payload_long.text)
		)
		nmea = '$'+nmea+genChecksum(nmea)
		print nmea
		# pAlt = float(self.parent.ids.payload_alt.text)


	def decDegToNMEA(deg):
		''' Converts from dec degrees to deg.decMinutes for NMEA '''
		deg, dMin = deg.split('.')
		minutes = str(float('0.'+dMin) * 60)[:6]
		if deg.startswith('-'):
			deg = deg[1:]
			suffix = ',W'
		else:
			suffix = ',N'
		return deg+minutes+suffix


	def genChecksum(sentence):
		''' Adds leading $ and trailing checksum to a custom NMEA string '''
	    checksum = 0
		for s in nmea:
			checksum ^= ord(s)
		return '*'+hex(checksum)[2:]


	def degToStepper(self, deg):
		''' Converts from degrees to stepper range, based on 1/16 microsteps '''
		# Assuming 0 degrees is straight ahead
		steps = (int)(deg * 48960 / 360)
		return steps


	def moveToCenter(self):
		self.moveBoth(0, 0)


	def moveAz(self, deg):
		pass


	def moveEle(self, deg):
		pass


	def moveBoth(self, eleDeg, aziDeg):
		if self.connected and not self.parent.ids.motor_switchstop.active:
			aziSteps = self.degToStepper(aziDeg)
			eleSteps = self.degToStepper(eleDeg)
			cmd = "<{},{}>".format(eleSteps, aziSteps)
			if self.prevAziSteps != aziSteps and self.prevEleSteps != eleSteps:
				print(cmd)
			cmd = str.encode(cmd)
			self.usb.write(cmd)


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
