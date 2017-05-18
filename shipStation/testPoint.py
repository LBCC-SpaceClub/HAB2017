import Arduino
import ServoControl
# import Database
import time
import geomag

tracker = None
servos = None
USELOCAL = True

# Connect to the local arduino
try:
    tracker = Arduino.Arduino()
except IOError:
    print "Please connect an appropriate Arduino and try again."
    exit(2)

# Connect to the servo controllers
try:
    servos = ServoControl.ServoControl()
except IOError:
    print "Could not find servos, will write output to screen instead."

# Connect to MSU's irridium database
# try:
    # target = Database.DatabaseConnect()
# except:
    # print "Failure to connect to database."

while True:
    if USELOCAL:
        targetlatDeg = 44.587746
        targetlonDeg = -123.096098
        targetaltMeters = 200
        #lat = input('Enter latitude of target: ')
        #lon = input('Enter longitude of target: ')
        #alt = input('Enter altitude of target: ')
    else:
        target.update()
    tracker.update()

    bearing = ServoControl.bearing(
        tracker.latDeg, tracker.lonDeg, targetlatDeg, targetlonDeg
    )

    if(bearing):
        tracker.imuX = (tracker.imuX + 360)%360
        # print "The tracker heading is ", tracker.imuX
        declination = geomag.declination(
            dlat=tracker.latDeg, dlon=tracker.lonDeg, h=tracker.altMeters
        )
        # print "Magnetic declination is ", declination
        tracker.imuX = (tracker.imuX + declination) % 360
        print "Tracker heading after declination: ", tracker.imuX

        print "The solution bearing is: ", bearing
        solution = (bearing - tracker.imuX + 360) % 360
        print "Tracker needs to move ", solution
        print "In servo, that's ", ServoControl.degToServo(solution)
        print tracker.imuX, tracker.imuMag, tracker.imuSys
        time.sleep(5)
