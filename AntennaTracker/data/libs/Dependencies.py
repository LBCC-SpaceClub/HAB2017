
'''
	Kivy standard library imports
'''
import kivy
from kivy.config import Config
kivy.config.Config.set('graphics','resizable', False) #config needs to be set before kivy.app is imported
from kivy.app import App
from time import time
from os.path import dirname, join
from kivy.lang import Builder
from kivy.properties import NumericProperty,StringProperty,BooleanProperty
from kivy.properties import ListProperty,ReferenceListProperty,ObjectProperty
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
from kivy.uix.rst import RstDocument
from kivy.clock import Clock, mainthread
from kivy.uix.videoplayer import VideoPlayer


'''
	Project imports
'''
from data.IntervalThread import *
from data.DatabaseThread import *
from data.ArduinoThread import *

from data.libs.MyKnob import *
from data.libs.garden.mapview import *