import pymysql.cursors
import configparser
import time

class DatabaseConnect():
    """ Fetch the latest GPS coordinates from MSU's irridium database """
    def __init__(self, cfg_file):
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
        self.lastChecked = None

    def update(self):
        ''' Read in new data from the database '''
        if time.time() - self.lastChecked > 30: # don't hammer db
            print "Checking the database..."
            self.gpsDate, self.gpsTime, self.latDeg, self.lonDeg, self.altMeters = parseData()
            self.lastChecked = time.time()

    def parseData(self):
        try:
            sql = self.db.cursor()
            query = "select gps_fltDate,gps_time,gps_lat,gps_long,gps_alt "\
                    "from gps order by pri_key DESC"
            sql.execute(query)
            try:
            	result = sql.fetchone()
            	return result
            except:
            	print("ERROR - failed to get data from database")
            	sql.close()
            	return
        except:
            print("ERROR - failed to parse data, check internet connection")

if __name__ == "__main__":
    print "Database is not a standalone Python program."
