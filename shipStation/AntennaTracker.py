from kivy.app import App
from kivy.lang import Builder
from kivy.factory import Factory
from kivy.animation import Animation
from kivy.clock import Clock, mainthread
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.properties import NumericProperty,ReferenceListProperty,ObjectProperty
import threading
import time
from Actions import *
from Database import *
from IntervalThread import *

configs = "Configs.ini"

class AntennaTracker(App):
  thread = IntervalThread()

  @thread.setInterval(5)
  def __queryDatabase(self):
  	self.db = DatabaseConnect(configs)
  	print(self.db.parseData())
  
  def build(self):
  	self.__queryDatabase()
  	self.load_kv('Components.kv')
  	return RootFrame()
  

#-------------------------START OF EXECUTION
if __name__ == "__main__":
  AntennaTracker().run()