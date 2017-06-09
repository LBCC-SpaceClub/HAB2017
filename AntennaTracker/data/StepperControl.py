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
		self.hasNewPayloadGps = False
		self.targetAziDeg = 0
		self.targetEleDeg = 0
		self.targetDistanceM = 0
		self.magneticVariation = 0

		# Connect to the arduino
		self.arduino = Arduino(parent)
		print("Steppers created!")

	def run(self):
		while self.running:
			# Simplify by using some temp variables
			stationConnected = self.parent.ids.station_connect.disabled
			stationManualButton = self.parent.ids.station_switchmanual.active
			payloadConnected = self.parent.ids.payload_connect.disabled
			payloadManualButton = self.parent.ids.payload_switchmanual.active

			# Manual mode currently undefined using stepper controller..
			if stationConnected:
				# Parse all available serial data
				self.update()
				self.updateGui()


			# Update the gui compasses if both gps positions available
			hasStationGps = (stationConnected or stationManualButton)
			hasPayloadGps = (payloadConnected or payloadManualButton)
			if (hasStationGps and hasPayloadGps):
				self.updateGuiCompass()

			# Consider only sending when necessary, like this:
			# newPayloadAvailable = self.lastSent < self.parent.ids.payload_time.text
			# if stationConnected and newPayloadAvailable:
			if stationConnected and hasPayloadGps:
				# Send the newest payload location to the arduino
				self.arduino.sendPayloadGps(self.magneticVariation)

			# No point updating faster than new data becomes available
			time.sleep(1)


	def update(self):
		while self.arduino.hasLines():
			line = self.arduino.getLine()
			if len(line) > 10:
				print('Received: ', line)
			try:
				if line.startswith('[IMU]'):
					self.parseIMU(line[5:])
				elif line.startswith('[TGPS]'):
					self.parseGPS(line[6:])
				elif line.startswith('[TIME]'):
					self.parseTime(line[6:])
				elif line.startswith('[SOL]'):
					self.parseSolution(line[5:])
			except Exception as e:
				print("Failed to parse a line: {}".format(e))
				print(line)


	def updateGui(self):
		if self.hasNewPayloadGps:
			self.parent.ids.station_lat.text = str(self.latDeg)
			self.parent.ids.station_long.text = str(self.lonDeg)
			self.parent.ids.station_alt.text = str(self.altMeters)
			self.parent.ids.station_trueHeading.text = str(self.imuX)
			self.parent.ids.station_time.text = str(self.gpsTime)
			self.magneticVariation = geomag.declination(
				dlat=self.latDeg, dlon=self.lonDeg, h=self.altMeters
			)
			self.hasNewPayloadGps = False


	def updateGuiCompass(self):
		# if self.parent.ids.station_switchmanual.active:
			# self.updateTargetSolution()
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
		stationManualButton = self.parent.ids.station_switchmanual.active
		# Don't parse local GPS strings if we're not using them
		if not stationManualButton:
			# print("Parsing GPS")
			line = line.split(',')
			self.latDeg = float(line[0])
			self.lonDeg = float(line[1])
			self.altMeters = float(line[2])
			if len(line) >= 4: # this field may be empty
				self.magneticVariation = float(line[3])
			self.hasNewPayloadGps = True


	def parseTime(self, line):
		line = line.split(',')
		self.gpsDate = line[0]
		self.gpsTime = line[1]


	def parseSolution(self, line):
		line = line.split(',')
		self.gpsDate = line[0]
		self.gpsTime = line[1]
		self.targetAziDeg = float(line[0])
		self.targetEleDeg = float(line[1])



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
		self.lastSentNmea = None
		self.lastSentTime = time.time()
		print("Connected to stepper arduino.")
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


	def sendPayloadGps(self, magVariation):
		'''
			$GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
			1    = UTC of Position
			2    = Latitude
			3    = N or S
			4    = Longitude
			5    = E or W
			6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
			7    = Number of satellites in use [not those in view]
			8    = Horizontal dilution of position
			9    = Antenna altitude above/below mean sea level (geoid)
			10   = Meters  (Antenna height unit)
			11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
			       mean sea level.  -=geoid is below WGS-84 ellipsoid)
			12   = Meters  (Units of geoidal separation)
			13   = Age in seconds since last update from diff. reference station
			14   = Diff. reference station ID#
			15   = Checksum
		'''

		nmea = "GPGGA,{now},{lat},{lon},1,{sats},0.0,{alt},M,{geoid},M,{mag},".format(
			now = time.strftime('%H%M%S')[:6], #HHMMSS.SS UTC
			lat = self.decDegToNMEA(self.parent.ids.payload_lat.text),
			lon = self.decDegToNMEA(self.parent.ids.payload_long.text),
			sats = '07',
			alt = self.parent.ids.payload_alt.text,
			geoid = self.parent.ids.payload_alt.text, # not sure how to calc?
			mag = magVariation
		)
		nmea = '${}*{:02X}\r'.format(nmea,self.genChecksum(nmea)).encode('utf-8')
		# No point resending the same string over and over without a delay
		if nmea != self.lastSentNmea or (time.time() - self.lastSentTime) > 5.0:
			# print("Sending: ", nmea)
			self.lastSentNmea = nmea
			self.lastSentTime = time.time()
			self.usb.write(nmea)


	def decDegToNMEA(self, deg):
		''' Converts from dec degrees to deg.decMinutes for NMEA '''
		deg, dMin = deg.split('.')
		# Clunky hack to convert from DDD.DDDD to DDDMM.MMMM
		dMin = float('0.' + dMin) * 60
		minutes = "{:02d}.{}".format(int(dMin), int(dMin*1000))
		if deg.startswith('-'):
			deg = deg[1:9]
			suffix = ',W'
		else:
			deg = deg[:7]
			suffix = ',N'
		return deg+minutes+suffix


	def genChecksum(self, sentence):
		''' Adds leading $ and trailing checksum to a custom NMEA string '''
		checksum = 0
		for s in sentence:
			checksum ^= ord(s)
		return checksum


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
