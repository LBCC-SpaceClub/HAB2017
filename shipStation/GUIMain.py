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

from GUIActions import *


class GUIMain(App):
    def build(self):
        self.load_kv('Components.kv')
        return RootFrame()


