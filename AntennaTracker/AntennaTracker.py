#!/usr/bin/python3
"""
	Authors:	Kyle Prouty 	<kyle@prouty.io>
								<proutyky@oregonstate.edu>
				Levi Willmeth 	<levi.willmeth@gmail.com>
"""
from data.libs.Dependencies import *


class RootLayout(FloatLayout):

	configs = "cfg/Configs.ini"
	interval_threadA = IntervalThread()
	interval_threadB = IntervalThread()
	interval_threadC = IntervalThread()
	interval_threadD = IntervalThread()
	x_value = NumericProperty(0.0)
	y_value = NumericProperty(0.0)
	rst_doc = StringProperty('')
	db_check = False
	arduino_check = False
	db_list = []
	arduino_list = []
	map_list = []
	map_oldcoords = []
	map_lat  = 44.5637806#OSU Lat/Long
	map_long = -123.2794442


	def __init__(self, **kwargs):
		super(RootLayout, self).__init__(**kwargs)
		Clock.schedule_once(self.run)



	##################################################
	###
	###		Start-up Setting
	###
	##################################################
	def run(self, args):
		self.updateConsole(" **WELCOME** setting initialized")
		self.ids.db_status.text = "Not Connected"
		self.ids.db_status.color = (1,0,0,1) #green(0,1,0,1)red(1,0,0,1)
		self.ids.ard_status.text = "Not Connected"
		self.ids.ard_status.color = (1,0,0,1)
		self.ids.eth_status.text = "Not Connected"
		self.ids.eth_status.color = (1,0,0,1)
		self.ids.payload_disconnect.disabled = True
		self.ids.station_disconnect.disabled = True
		self.payloadManualSwitch()
		self.stationManualSwitch()
		self.motorManualSwitch()
		self.mapUpdate()



	##################################################
	###
	###		Iridium Database Methods
	###
	##################################################
	def startIridiumDatabase(self):
		self.updateConsole(" **START** iridium database connection")
		try:
			self.ids.payload_connect.disabled = True
			self.db_check = True
			self.db_list.insert(0,DatabaseThread(self.configs))
			self.poolLogMessages()
			self.poolDatabaseStatus()
			self.db_list[0].start()
		except:
			self.updateConsole(" **ERROR** Could not run startIridiumDatabase")


	def stopIridiumDatabase(self):
		if(self.db_list):
			self.updateConsole(" **STOP** iridium database connection")
			self.ids.payload_lat.text = "0.0"
			self.ids.payload_long.text = "0.0"
			self.ids.payload_alt.text = ""
			self.ids.payload_date.text = ""
			self.ids.payload_time.text = ""
			self.db_list[0].connected = False
			self.db_list[0].stop = False #must happen before pop or thread wont get garbage collected
			self.db_list.pop(0)
			self.db_check = False
			self.ids.db_status.text = "Not Connected"
			self.ids.db_status.color = (1,0,0,1)
			self.ids.payload_disconnect.disabled = True
			self.ids.payload_connect.disabled = False



	##################################################
	###
	###		Arduino Methods
	###
	##################################################
	def startArduinoUSB(self):
		self.updateConsole(" **START** arduino usb connection")
		self.arduino_list.insert(0,ArduinoThread())
		self.arduino_check = True
		self.poolLogMessages()
		if(self.arduino_list):
			try:
				self.arduino_list[0].start()
			except:
				self.arduino_list.pop(0)
				return
			if(self.arduino_list[0].connected == True):
				self.ids.station_connect.disabled = True
				self.ids.station_disconnect.disabled = False
				self.checkArduinoStatus()


	def stopArduinoUSB(self):
		if(self.arduino_list):
			self.updateConsole(" **STOP** arduino usb")
			self.ids.station_lat.text = ""
			self.ids.station_long.text = ""
			self.ids.station_alt.text = ""
			self.ids.station_date.text = ""
			self.ids.station_time.text = ""
			self.arduino_list[0].isConnected = False
			self.arduino_list[0].stop = False #must happen before pop or thread wont get garbage collected
			self.arduino_list.pop(0)
			self.arduino_check = False
			self.ids.ard_status.text = "Not Connected"
			self.ids.ard_status.color = (1,0,0,1)
			self.ids.station_disconnect.disabled = True
			self.ids.station_connect.disabled = False



	##################################################
	###
	###		Payload Methods
	###
	##################################################
	def payloadConnect(self):
		if(self.ids.cbox_database.active):
			self.startIridiumDatabase()


	def payloadDisconnect(self):
		self.stopIridiumDatabase()


	def payloadSetManualValues(self):
		self.updateConsole(" **SET** payload ("+self.ids.payload_lat.text+", "
			+self.ids.payload_long.text+", "+self.ids.payload_alt.text+")")


	def payloadSetGPSValues(self):
		self.updateConsole(" **SET** map gps (Lat: "+self.ids.payload_lat.text+", Long: "
			+self.ids.payload_long.text+")")
		try:
			self.map_lat = float(self.ids.payload_lat.text)
			self.map_long = float(self.ids.payload_long.text)
		except:
			pass
		self.mapUpdate()


	def payloadManualSwitch(self):
		if(self.ids.payload_switchmanual.active):
			self.updateConsole(" **MODE** manual payload")
			self.ids.payload_setvalues.disabled = False
			self.ids.payload_setgps.disabled = False
			self.ids.payload_lat.readonly = False
			self.ids.payload_long.readonly = False
			self.ids.payload_alt.readonly = False
			self.ids.payload_lat_lbl.color = (0, 1, 1, 1)
			self.ids.payload_long_lbl.color = (0, 1, 1, 1)
			self.ids.payload_alt_lbl.color = (0, 1, 1, 1)
			self.ids.payload_connect.disabled = True
			self.ids.payload_disconnect.disabled = True
			self.stopIridiumDatabase()
		else:
			self.updateConsole(" **MODE** auto payload")
			self.ids.payload_setvalues.disabled = True
			self.ids.payload_setgps.disabled = True
			self.ids.payload_lat.readonly = True
			self.ids.payload_long.readonly = True
			self.ids.payload_alt.readonly = True
			self.ids.payload_lat_lbl.color = (1, 1, 1, 1)
			self.ids.payload_long_lbl.color = (1, 1, 1, 1)
			self.ids.payload_alt_lbl.color = (1, 1, 1, 1)
			self.ids.payload_connect.disabled = False



	##################################################
	###
	###		Station Methods
	###
	##################################################
	def stationConnect(self):
		if(self.ids.cbox_arduino_usb.active):
			self.startArduinoUSB()


	def stationDisconnect(self):
		self.stopArduinoUSB()


	def stationSetManualValues(self):
		self.updateConsole(" **SET** station ("+self.ids.station_lat.text+", "
			+self.ids.station_long.text+", "+self.ids.station_alt.text+")")


	def stationManualSwitch(self):
		if(self.ids.station_switchmanual.active):
			self.updateConsole(" **MODE** manual station")
			self.ids.station_setvalues.disabled = False
			self.ids.station_lat.readonly = False
			self.ids.station_long.readonly = False
			self.ids.station_alt.readonly = False
			self.ids.station_lat_lbl.color = (0, 1, 1, 1)
			self.ids.station_long_lbl.color = (0, 1, 1, 1)
			self.ids.station_alt_lbl.color = (0, 1, 1, 1)
			self.ids.station_connect.disabled = True
			self.ids.station_disconnect.disabled = True
		else:
			self.updateConsole(" **MODE** auto station")
			self.ids.station_setvalues.disabled = True
			self.ids.station_lat.readonly = True
			self.ids.station_long.readonly = True
			self.ids.station_alt.readonly = True
			self.ids.station_lat_lbl.color = (1, 1, 1, 1)
			self.ids.station_long_lbl.color = (1, 1, 1, 1)
			self.ids.station_alt_lbl.color = (1, 1, 1, 1)
			self.ids.station_connect.disabled = False



	##################################################
	###
	###		Map Methods
	###
	##################################################
	def mapUpdate(self):
		#check cords to see if they changed..if they have then print update
		try:
			self.updateConsole(" **UPDATE** map latitude and longitude.")
			self.ids.mapview.center_on(self.map_lat, self.map_long)
			if self.map_list:
				self.ids.mapview.remove_marker(self.map_list[0])
				self.map_list.pop(0)
			if not self.map_list:
				try:
					self.map_list.insert(0, MapMarker(lon=self.map_long, lat=self.map_lat))
					self.ids.mapview.add_marker(self.map_list[0])
				except:
					pass
		except:
			self.updateConsole(" **ERROR** invalid latitude and longitude")



	##################################################
	###
	###		Motor Control Methods
	###
	##################################################
	def motorStopSwitch(self):
		if(self.ids.motor_switchstop.active):
			self.updateConsole(" **STOP** motors")


	def motorManualSwitch(self):
		if(self.ids.motor_switchmanual.active):
			self.updateConsole(" **MODE** manual motor control")
			self.ids.motor_sliderX.disabled = False
			self.ids.motor_sliderX_text.disabled = False
			self.ids.motor_sliderY.disabled = False
			self.ids.motor_sliderY_text.disabled = False
		else:
			self.updateConsole(" **MODE** auto motor control")
			self.ids.motor_sliderX.disabled = True
			self.ids.motor_sliderX_text.disabled = True
			self.ids.motor_sliderY.disabled = True
			self.ids.motor_sliderY_text.disabled = True



	##################################################
	###
	###		Helper Methods
	###
	##################################################
	def updateConsole(self, text):
		self.rst_doc=(self.rst_doc+"\n"+"> "+text+"\n")

	def sliderValidate(self, value):
		if(value > 360.0):
			return 360.0
		else:
			return value

	def truncate(self,val):
		f = '%.12f' % val
		a,b,c = f.partition('.')
		return '.'.join([a, (c+'0'*2)[:2]])

	def aboutPopup(self):
		popup = AboutPopup()
		popup.open()

	def exitPopup(self):
		popup = ExitPopup()
		popup.open()



	##################################################
	###
	###		Pooling Threads
	###
	##################################################
	## Pooling Log Messages
	@interval_threadA.setInterval(1)
	def poolLogMessages(self):
		if(self.db_check):
			if (self.db_list[0].getLog() ==""):
				pass
			else:
				self.updateConsole(self.db_list[0].getLog())
				self.db_list[0].clearLog()
		if(self.arduino_check):
			if(self.arduino_list[0].getLog() ==""):
				pass
			else:
				self.updateConsole(self.arduino_list[0].getLog())
				self.arduino_list[0].clearLog()


	## Pooling the status of connections / updates DB values
	@interval_threadB.setInterval(5)
	def poolDatabaseStatus(self):
		if(self.db_list):
			if(self.db_list[0].connected):
				self.db_list[0].update()
				self.ids.db_status.text = "Connected"
				self.ids.db_status.color = (0,1,0,1)
				self.ids.payload_disconnect.disabled = False
				if self.ids.payload_time.text != str(self.db_list[0].gpsTime):
					self.ids.payload_lat.text = self.db_list[0].latDeg
					self.ids.payload_long.text = self.db_list[0].lonDeg
					self.ids.payload_alt.text = self.db_list[0].altMeters
					self.ids.payload_date.text = str(self.db_list[0].gpsDate)
					self.ids.payload_time.text = str(self.db_list[0].gpsTime)
					self.mapUpdate()
					try:
						self.map_lat = float(self.db_list[0].latDeg)
						self.map_long = float(self.db_list[0].lonDeg)
					except:
						self.updateConsole(
							" **ERROR** Could not update map coordinates."
						)
			else:
				#self.db_list.pop(0)
				self.ids.payload_connect.disabled = True
				self.ids.payload_disconnect.disabled = True


	@interval_threadC.setInterval(1)
	def checkArduinoStatus(self):
		if(self.arduino_check):
			if(self.arduino_list[0].isConnected):
				self.ids.ard_status.text = "Connected"
				self.ids.ard_status.color = (0,1,0,1)
				self.ids.station_lat.text = self.arduino_list[0].latDeg
				self.ids.station_long.text = self.arduino_list[0].lonDeg
				self.ids.station_alt.text = self.arduino_list[0].altMeters
				self.ids.station_date.text = str(self.arduino_list[0].gpsDate)
				self.ids.station_time.text = str(self.arduino_list[0].gpsTime)
				self.arduino_list[0].servoMoveTilt(x_value, self.arduino_list[0].usb)


#-------------------------------#



#-------------------------------#
class AboutPopup(Popup):
	def closePopup(self, *args):
		self.dismiss()


#-------------------------------#
class ExitPopup(Popup):
	def closePopup(self, *args):
		self.dismiss()


#---------------------------START OF EXECUTION
class AntennaTracker(App):
	def build(self):
		self.title = 'AntennaTracker'
		self.load_kv('layout/layout.kv')
		Window.size = (1280,900)
		return RootLayout()

if __name__ == '__main__':
    AntennaTracker().run()
