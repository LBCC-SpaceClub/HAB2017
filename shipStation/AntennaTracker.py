from kivy.app import App
from kivy.core.window import Window
from kivy.uix.widget import Widget
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.clock import Clock, mainthread
from kivy.properties import NumericProperty,ReferenceListProperty,ObjectProperty
from Database import *
from IntervalThread import *


class LeftFrame(BoxLayout):

	thread = IntervalThread()
	configs = "Configs.ini"
	
	def __init__(self, **kwargs):
		super(LeftFrame, self).__init__(**kwargs)
		Clock.schedule_once(self._run)

	def _run(self, args):
		self.updateDatabase()

	@thread.setInterval(.5)
	def updateDatabase(self):
		db = DatabaseConnect(self.configs)
		data = db.parseData()
		self.ids.lbl_lat.text = data["gps_lat"]
		self.ids.lbl_long.text = data["gps_long"]
		self.ids.lbl_alt.text = data["gps_alt"]
		self.ids.lbl_date.text = str(data["gps_fltDate"])
		self.ids.lbl_time.text = str(data["gps_time"])




class Root(BoxLayout):
	pass




#---------------------------START OF EXECUTION
class AntennaTracker(App):
	def build(self):
		self.load_kv('Layout.kv')
		Window.size = (1024,900)
		return Root()

if __name__ == "__main__":
  AntennaTracker().run()