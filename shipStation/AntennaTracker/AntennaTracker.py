from time import time
from kivy.app import App
from os.path import dirname, join
from kivy.lang import Builder
from kivy.config import Config 
from kivy.properties import NumericProperty, StringProperty, BooleanProperty, ListProperty,ReferenceListProperty,ObjectProperty
from kivy.clock import Clock
from kivy.animation import Animation
from kivy.uix.screenmanager import Screen
from kivy.core.window import Window
from kivy.uix.widget import Widget
from kivy.uix.bubble import Bubble
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.button import Button
from kivy.clock import Clock, mainthread

from data.IntervalThread import *
from data.DatabaseThread import *
'''import Arduino '''


class MainLayout(FloatLayout):

	configs = "cfg/Configs.ini"
	interval_thread = IntervalThread()
	db_thread = DatabaseThread(configs)

	
	def __init__(self, **kwargs):
		super(MainLayout, self).__init__(**kwargs)
		Clock.schedule_once(self.run)


	def updateConsole(self, text):
		self.ids.consolelog.text=self.ids.consolelog.text+"\n"+">\t\t"+text


	### INITIAL SETTING ###
	def run(self, args):
		self.ids.db_status.text = "Not Connected"
		self.ids.db_status.color = (1,0,0,1) #green(0,1,0,1)red(1,0,0,1)
		self.ids.ard_status.text = "Not Connected"
		self.ids.ard_status.color = (1,0,0,1)
		self.ids.eth_status.text = "Not Connected"
		self.ids.eth_status.color = (1,0,0,1)
		self.ids.reconnect_payload.disabled = True
		self.updateConsole("Welcome, setting initialized!")
		self.checkAutoMove()


	def startIrridiumDatabase(self):
		self.db_thread.start()
		self.ids.connect_payload.disabled = True
		self.ids.reconnect_payload.disabled = False
		self.checkDBStatus()
		self.updateConsole("Starting irridium database connection...")


	def stopIrridiumDatabase(self):
		self.db_thread.stop()


	def payloadConnect(self):
		if(self.ids.cbox_database.active):
			self.startIrridiumDatabase()


	def payloadReconnect(self):
		self.stopIrridiumDatabase()


	def stationConnect(self):
		pass


	def stationReconnect(self):
		pass


	def checkAutoMove(self):
		if(self.ids.switch_automove.active):
			self.ids.input_automove.disabled = True
			self.ids.button_automove.disabled = True
		else:
			self.ids.input_automove.disabled = False
			self.ids.button_automove.disabled = False


	## Function sits on a Thread in background and pools the status of connections / updates DB values
	@interval_thread.setInterval(15)
	def checkDBStatus(self):
		if(self.db_thread.isConnected):
			self.ids.db_status.text = "Connected"
			self.ids.db_status.color = (0,1,0,1)
			self.updateConsole("Parsing database data...")

			self.ids.payload_lat.text = self.db_thread.latDeg
			self.ids.payload_long.text = self.db_thread.lonDeg
			self.ids.payload_alt.text = self.db_thread.altMeters
			self.ids.payload_date.text = str(self.db_thread.gpsDate)
			self.ids.payload_time.text = str(self.db_thread.gpsTime)


	def confirmExit(self):
		popup = ExitPopup()
		popup.open()




class ExitPopup(Popup):
	def closePopup(self, *args):
		self.dismiss()


#---------------------------START OF EXECUTION
class AntennaTracker(App):	

	def build(self):
		self.title = 'AntennaTracker'
		self.load_kv('layout/layout.kv')
		Config.set('graphics','resizable', False) #cant get this to work
		Window.size = (1280,1024)
		return MainLayout()

	def myexit(self):
		app.stop()
	




if __name__ == '__main__':
    AntennaTracker().run()