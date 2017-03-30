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
from Database import DBConnect



configs = "Configs.ini"
db = DBConnect(configs)
print(db.parseData())



class AntennaTracker(App):
    def build(self):
        self.load_kv('Components.kv')
        return RootFrame()


if __name__ == "__main__":
    AntennaTracker().run()