import pymysql.cursors
import configparser


class DBConnect():
  def __init__(self, cfg_file):
    try:
  	  cfg = configparser.ConfigParser()
  	  cfg.read(cfg_file)
    except:
  	  print("ERROR - could not read config file")

    try:
      self.db = pymysql.connect(host=cfg["MySQL"]["Host"],
	  	user=cfg["MySQL"]["User"],
	  	password=cfg["MySQL"]["Password"],
	  	db=cfg["MySQL"]["Database"],
	  	charset='utf8mb4',
	  	cursorclass=pymysql.cursors.DictCursor)
    except:
      print("ERROR - failed to connect to MySQL database")


  def parseData(self):
    try:
      sql = self.db.cursor()
      query = "select gps_fltDate,gps_time,gps_lat,gps_long,gps_alt from gps order by pri_key DESC"
      sql.execute(query)

      try:
      	result = sql.fetchone()
      	return result
      except:
      	print("ERROR - failed to get data from database")
      	sql.close()
      	return 
    except:
      print("ERROR - failed to parse data, no connection")

