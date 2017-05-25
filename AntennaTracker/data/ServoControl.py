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
			if not self.parent.ids.station_switchmanual.active:
				self.update()
				self.updateMain()
			self.updateGuiCompass()
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
				print("Exception on line from arduino")
				# self.parent.updateConsole(" **ERROR** Parsing line from arduino")

	def updateMain(self):
		self.parent.ids.station_lat.text = str(self.latDeg)
		self.parent.ids.station_long.text = str(self.lonDeg)
		self.parent.ids.station_alt.text = str(self.altMeters)
		self.parent.ids.station_trueHeading.text = str(self.imuX)
		self.parent.ids.station_time.text = str(self.gpsTime)


	def updateGuiCompass(self):
		b = self.getAzAngle()
		self.parent.x_value = b

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


	def getAzAngle(self):
		''' Find degrees from true North at tracker, to payload '''
		# Regardless of source, all payload GPS values go into the GUI
		pLat = self.parent.ids.payload_lat.text
		pLon = self.parent.ids.payload_long.text

		if self.parent.ids.station_switchmanual.active:
			# Use manually entered GPS values
			tLat = self.parent.ids.station_lat.text
			tLon = self.parent.ids.station_long.text
		else:
			# Use latest values (should be the same?)
			tLat = self.latDeg
			tLon = self.lonDeg
		print('Using values: tLat {} tLon {} pLat {} pLon {}.\n'.format(
			tLat, tLon, pLat, pLon
		))
		return self.bearing(tLat, tLon, pLat, pLon)



	def bearing(self, trackerLat, trackerLon, payloadLat, payloadLon):
		''' Returns bearing in degrees, from tracker to payload '''
		# http://www.movable-type.co.uk/scripts/latlong.html
		'''
		# Formula: θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
		# where φ1,λ1 is the start point,
		# φ2,λ2 the end point and Δλ is the difference in longitude
		'''

		try:
			startLat = math.radians(float(trackerLat))
			startLong = math.radians(float(trackerLon))
			endLat = math.radians(float(payloadLat))
			endLong = math.radians(float(payloadLon))

			dLong = endLong - startLong

			dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
			if abs(dLong) > math.pi:
				if dLong > 0.0:
					dLong = -(2.0 * math.pi - dLong)
				else:
					dLong = (2.0 * math.pi + dLong)

			rads = math.atan2(dLong, dPhi)
			degs = (math.degrees(rads) + 360.0) % 360.0;

			declination = geomag.declination(dlat=startLat, dlon=startLong)
			d2 = geomag.declination(dlat=self.latDeg, dlon=self.lonDeg)

			print('Bearing in radians: {:f} and degrees: {:f} + declination: {:f} + d2: {:f}'.format(rads, degs, declination, d2))
			return degs+declination

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
		self.parent = parent
		self.connected = False
		self.servoBaud = 9600
		self.servoTimeout = 1
		self.servoCOM = self.findComPort()
		# Servo ranges
		self.moveCommand = 0xFF
		self.minAz = 0
		self.maxAz = 255
		self.minEle = 70
		self.maxEle = 123
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
			if(position < self.minAz):          #80 degrees upper limit
				cmd = [self.moveCommand, self.tiltChannel, chr(self.minAz)]
			elif(position > self.maxAz):       #5 degrees lower limit
				cmd = [self.moveCommand, self.tiltChannel, chr(self.maxAz)]
			else:
				cmd = [self.moveCommand, self.tiltChannel, chr(position)]
			self.usb.write(cmd)


		def moveEle(position):
			if(position < self.minAz):
				cmd = [self.moveCommand, self.panChannel, chr(self.minAz)]
			elif(position > self.maxAz):
				cmd = [self.moveCommand, self.panChannel, chr(self.maxAz)]
			else:
				cmd = [self.moveCommand, self.panChannel, chr(position)]
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
