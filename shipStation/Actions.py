from AntennaTracker import *


class RootFrame(BoxLayout):
    def update(self):
    	self.ids.lbl.text = "Pressed"

    def update2(self):
    	self.ids.lbl.text = "Released"