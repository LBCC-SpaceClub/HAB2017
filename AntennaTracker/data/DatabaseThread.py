from threading import Thread
from random import randint
import pymysql.cursors
import configparser
import time


class DatabaseThread(Thread):

	def __init__(self, cfg_file):
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
		self.log = ""
		self.connected = False
		

	def run(self):
		if(self.connectToDB()):
			self.setLog("SUCCESS iridium database connected, parsing data")
			self.connected = True
			
			while(self.stop):
				self.update()
		else:
			self.setLog("ERROR failed to connect to MySQL database")
			self.connected = False


	def connectToDB(self):
		try:
			cfg = configparser.ConfigParser()
			cfg.read(self.cfg_file)
		except:
			self.setLog("ERROR could not read database config file")
			return False
		
		try:
			self.db = pymysql.connect(
				host=cfg["MySQL"]["Host"],
				user=cfg["MySQL"]["User"],
				password=cfg["MySQL"]["Password"],
				db=cfg["MySQL"]["Database"],
				charset='utf8mb4',
				cursorclass=pymysql.cursors.DictCursor
			)
			return True
		except:
			return False


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
				if not self.querysuccess:
					self.setLog("UPDATE parsing data successful")
					self.querysuccess = True
				else:
					self.setLog("UPDATE Iridium database data")
				return result
			except:
				self.setLog("ERROR failed to get data from database")
				self.querysuccess = False
				sql.close()
				return
		except:
			self.setLog("ERROR failed to parse data, check internet connection")


	def setLog(self, txt):
		self.log= self.log+""+txt


	def getLog(self):
		if(self.log != ""):
			return self.log
		else:
			return ""

	def clearLog(self):
		self.log = ""
