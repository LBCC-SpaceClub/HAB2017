import Arduino
import ServoControl
import time
import geomag

tracker = Arduino.Arduino()
servos = ServoControl.ServoControl()

while True:
    lat = 44.602255
    lon = -123.130668
    alt = 500
    #lat = input('Enter latitude of target: ')
    #lon = input('Enter longitude of target: ')
    #alt = input('Enter altitude of target: ')

    tracker.update()
    # print "{} is type {}".format(tracker.latDeg, type(tracker.latDeg)
    bearing = ServoControl.bearing(tracker.latDeg, tracker.lonDeg, lat, lon)
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
        time.sleep(1)
