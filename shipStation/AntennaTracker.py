from kivy.app import App
from kivy.core.window import Window
from Layout import *

#---------------------------START OF EXECUTION
class AntennaTracker(App):
	def build(self):
		self.load_kv('Components.kv')
		Window.size = (1024,900)
		return Root()

if __name__ == "__main__":
  AntennaTracker().run()