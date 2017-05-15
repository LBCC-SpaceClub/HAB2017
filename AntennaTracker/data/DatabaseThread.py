from threading import Thread
from random import randint
import pymysql.cursors
import configparser
import time
 
 
class DatabaseThread(Thread):

	def __init__(self, cfg_file):
		Thread.__init__(self)
		self._stop_event = False
		
		try:
			cfg = configparser.ConfigParser()
			cfg.read(cfg_file)
		except:
			print("ERROR - could not read database config file.")
		try:
			self.db = pymysql.connect(
				host=cfg["MySQL"]["Host"],
				user=cfg["MySQL"]["User"],
				password=cfg["MySQL"]["Password"],
				db=cfg["MySQL"]["Database"],
				charset='utf8mb4',
				cursorclass=pymysql.cursors.DictCursor
			)
		except:
			print("ERROR - failed to connect to MySQL database")
		self.gpsTime = None
		self.gpsDate = None
		self.latDeg = None
		self.lonDeg = None
		self.altMeters = None
		self.lastChecked = 0
		self.connected = False
		self.daemon = True #stops thread on app exit, important
		self.log = ""


	def stop(self):
		self._stop_event = True
		self.connected = False

	
	def stopped(self):
		return self._stop_event

	
	def run(self):
		while(1):
			self.update()

	
	def isConnected(self):
		return self.connected

	
	def setLog(self, txt):
		self.log= self.log+" "+txt

	
	def getLog(self):
		if(self.log != ""):
			return self.log
		else:
			return ""
	
	def clearLog(self):
		self.log = ""


	def update(self):
		''' Read in new data from the database '''
		if time.time() - self.lastChecked > 30: # don't hammer db			
			data = self.parseData()
			self.latDeg = data["gps_lat"]
			self.lonDeg = data["gps_long"]
			self.altMeters = data["gps_alt"]
			self.gpsDate = str(data["gps_fltDate"])
			self.gpsTime = str(data["gps_time"])
			self.lastChecked = time.time()
			

	def parseData(self):
		try:
			sql = self.db.cursor()
			query = "select gps_fltDate,gps_time,gps_lat,gps_long,gps_alt from gps order by pri_key DESC"
			sql.execute(query)
			try:
				result = sql.fetchone()
				if not self.connected:
					self.setLog("Irridium database connected successfully!")
				else:
					self.setLog("Database query successful!  Updating data..")
				self.connected = True
				return result
			except:
				self.setLog("ERROR - failed to get data from database")
				self.connected = False
				sql.close()
				return
		except:
			self.setLog("ERROR - failed to parse data, check internet connection")