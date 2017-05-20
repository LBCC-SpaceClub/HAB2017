import math
from math import radians, degrees, sin, cos, atan2, tan
import serial
import requests
import time


class ServoControl():

	panOffset = 0     # + right, - left
	tiltOffset = 0    # + raise, - lower
	moveCommand = 0xFF
	minPan = 0
	maxPan = 255
	minTilt = 70
	maxTilt = 123
	panChannel = 1
	tiltChannel = 0

	def __init__(self):
		self.log = ""


	def configServos(self, usb):
		self.setLog(" **UPDATE** configuring servos...")
		accelCommand = 0x89		#Set acceleration rate for both servos
		tiltAccel = 1
		panAccel = 1
		setAccel = [accelCommand,self.tiltChannel,tiltAccel,0]
		usb.write(setAccel)
		setAccel = [accelCommand,self.panChannel,panAccel,0]
		usb.write(setAccel)
		speedCommand = 0x87		  #Set rotation rate for both servos
		tiltSpeed = 1
		panSpeed = 3
		setSpeed = [speedCommand,self.tiltChannel,tiltSpeed,0]
		usb.write(setSpeed)
		setSpeed = [speedCommand,self.panChannel,panSpeed,0]
		usb.write(setSpeed)



	##################################################
	### 
	###     Move Servos Methods
	###
	##################################################
	def moveToCenterPos(self, usb):
		moveTiltServo(127, usb)
		movePanServo(127, usb)


	def moveTiltServo(self, degrees, usb):
		degInServo = 254.0 / 360 * degrees
		if(degInServo < self.minTilt):
		    degInServo = self.minTilt
		elif(degInServo > self.maxTilt):
		    degInServo = self.maxTilt
		degInServo = int(round(degInServo))
		moveTilt = [self.moveCommand, self.tiltChannel, chr(degInServo)]
		usb.write(degInServo)



	def movePanServo(self, position, usb):
		movePan = [moveCommand,panChannel,chr(255-position)]
		usb.write(movePan)


	def degToServo(d): 
		if d >= 360:
			d = d % 360
		if d < 180:
			res = int(round(127 - d*(255.0/360.0)))
		else:
			d = 360 - d
			res = int(round(127 + d*(255.0/360.0)))
		return res



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