import kivy
from kivy.config import Config 
kivy.config.Config.set('graphics','resizable', False) #config needs to be set before kivy.app is imported
from kivy.app import App
from time import time
from os.path import dirname, join
from kivy.lang import Builder
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


'''

I can put a switch in so that you can switch for manual to auto for sources and 
	then it would enable or disable the radio button of a source and enable or disable 
	inserting text and turn on or off a button to submit

yeah I can have a motor off button


Hmm ok let's see.
1) Radio buttons, for me the click area is above the graphic, so if I click on the middle button, the bottom button is selected.
2) Since some of the buttons aren't implemented yet, is it possible to disable them? If not, it's ok.
3) Calculations section should have a couple fields.
4) Automove should get lat/long/alt.
'''


class RootLayout(FloatLayout):

	configs = "cfg/Configs.ini"
	interval_threadA = IntervalThread()
	interval_threadB = IntervalThread()
	db_thread = DatabaseThread(configs)

	
	def __init__(self, **kwargs):
		super(RootLayout, self).__init__(**kwargs)
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
		#self.checkAutoMove()
		self.poolLogMessages()


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


	# def checkAutoMove(self):
	# 	if(self.ids.switch_automove.active):
	# 		self.ids.input_automove.disabled = True
	# 		self.ids.button_automove.disabled = True
	# 	else:
	# 		self.ids.input_automove.disabled = False
	# 		self.ids.button_automove.disabled = False

	
	@interval_threadB.setInterval(1)
	def poolLogMessages(self):
		if (self.db_thread.getLog() ==""):
			pass
		else:
			self.updateConsole(self.db_thread.getLog())
			self.db_thread.clearLog()


	## Function sits on a Thread in background and pools the status of connections / updates DB values
	@interval_threadA.setInterval(10)
	def checkDBStatus(self):
		if(self.db_thread.isConnected):
			self.ids.db_status.text = "Connected"
			self.ids.db_status.color = (0,1,0,1)

			self.ids.payload_lat.text = self.db_thread.latDeg
			self.ids.payload_long.text = self.db_thread.lonDeg
			self.ids.payload_alt.text = self.db_thread.altMeters
			self.ids.payload_date.text = str(self.db_thread.gpsDate)
			self.ids.payload_time.text = str(self.db_thread.gpsTime)


	def confirmExit(self):
		popup = ExitPopup()
		popup.open()



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