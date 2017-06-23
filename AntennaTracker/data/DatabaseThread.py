from threading import Thread
from random import randint
import pymysql.cursors
import configparser
import time


class DatabaseThread(Thread):

	def __init__(self, parent, cfg_file):
		Thread.__init__(self)
		self.cfg_file = cfg_file
		self.stop = True
		self.gpsTime = ""
		self.gpsDate = ""
		self.latDeg = ""
		self.lonDeg = ""
		self.altMeters = ""
		self.lastChecked = 0
		self.querysuccess = False
		self.daemon = True #stops thread on app exit, important
		self.parent = parent
		self.connected = self.connectToDB() #check connection

		#print("closing db")


	def __del__(self):
		# Clean up the db connection when this thread exits
		#print("db thread exiting")
		if self.db:
			self.db.close()
		print("closing db thread")

	def run(self):
		while self.connected:
			if not self.parent.ids.payload_switchmanual.active:
				self.update()
			time.sleep(5)


	def connectToDB(self):
		try:	# to open config file
			self.cfg = configparser.ConfigParser()
			self.cfg.read(self.cfg_file)
		except:
			self.parent.updateConsole(" **ERROR** could not read /cfg/Configs.ini")
			return False

		try:	# to connect to db
			self.db = pymysql.connect(
				host= self.cfg["MySQL"]["Host"],
				user= self.cfg["MySQL"]["User"],
				password= self.cfg["MySQL"]["Password"],
				db= self.cfg["MySQL"]["Database"],
				charset='utf8mb4',
				cursorclass=pymysql.cursors.DictCursor
			)
			#print("connecting to db")
			return True
		except:
			self.parent.updateConsole(" **ERROR** could not connect to iridium database")
			return False


	def update(self):
		#new connection to db for this update
		if time.time() - self.lastChecked > 30: # don't hammer db
			self.connectToDB()
			#print("Checking Database")
			try:
				with self.db.cursor() as sql:
					sql.execute(self.cfg["MySQL"]["Query"])
					data = sql.fetchone()
					#print(data)
					try:
						self.latDeg = data["gps_lat"]
						self.lonDeg = data["gps_long"]
						self.altMeters = data["gps_alt"]
						self.gpsDate = str(data["gps_fltDate"])
						self.gpsTime = str(data["gps_time"])
						#print("recorded db data")
						self.lastChecked = time.time()
						self.updateparent()
						self.db.close()
						#print("closing db")

					except:
						# a botched parse deserves a smaller delay
						self.lastChecked = time.time() - 20
						self.parent.updateConsole(" **ERROR** failed to parse line from database")
						print("**ERROR** failed to parse line from database")
			except:
				self.parent.updateConsole(" **ERROR** failed to fetch data from iridium database")
				print("**ERROR** failed to fetch data from iridium database")


	def updateparent(self):
		self.parent.ids.payload_lat.text = self.latDeg
		self.parent.ids.payload_long.text = self.lonDeg
		self.parent.ids.payload_alt.text = self.altMeters
		self.parent.ids.payload_date.text = str(self.gpsDate)
		self.parent.ids.payload_time.text = str(self.gpsTime)
		#print("updated db text")
		self.parent.payloadSetGPSValues()
		self.parent.updateConsole(" **UPDATE** updated location from iridium database")
