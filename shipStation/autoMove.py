import math
import serial

#Ground station location
groundLat = 0.00
groundLon = 0.00
groundAlt = 0.00
centerBear = 0.00
antennaBear = 0.00
antennaEle = 0.00
calibrationGoal = 8
moveIncrement = 2.0

#Serial Settings
s = serial.Serial()
servoCOM = "COM3"
servoBaud = 9600
servoTimeout = 0.5
arduinoCOM = "COM5"
arduinoBaud = 115200
arduinoTimeout = 5

# use these to manually tweak the tracking (ie if its still off a little after aligning)
panOffset = 0          # increase to turn right, decrease to turn left
tiltOffset = 0          # increase to raise, decrease to lower
trackTiltOffset = 0.0
trackBearOffset = 0.0
moveCommand = 0xFF
accelCommand = 0x89
speedCommand = 0x87

# Shouldn't need to change these unless you change to some exotic servos
servo_min = 0
servo_max = 254

def configServos():
    ''' Set the default movement parameters of the servos '''
    panChannel = 1
    panRange = 360
    panAccel = 1
    panSpeed = 3
    # change the movement speed etc of ubiquity tilt servo
    tiltChannel = 0
    tiltAccel = 1
    tiltSpeed = 1
    print "Setting servo configurations."
    s = serial.Serial(str(servoCOM), baudrate = servoBaud, timeout = servoTimeout)
    #Set acceleration for both servos
    setAccel = [accelCommand,tiltChannel,tiltAccel,0]
    s.write(setAccel)
    setAccel = [accelCommand,panChannel,panAccel,0]
    s.write(setAccel)
    #Set rotation speed for both servos
    setSpeed = [speedCommand,tiltChannel,tiltSpeed,0]
    s.write(setSpeed)
    setSpeed = [speedCommand,panChannel,panSpeed,0]
    s.write(setSpeed)
    s.close()

def moveToCenterPos():
    ''' Send servos to their center position '''
    print "Starting serial communication with",servoCOM
    moveTiltServo(127)
    movePanServo(127)
    print "Move to Center Command Sent via", servoCOM

def moveTiltServo(position):
    ''' Move the horizontal servo to a given position '''
    global antennaEle
    print "\tMove Tilt: ", float(position)
    s = serial.Serial(str(servoCOM), baudrate = servoBaud, timeout = servoTimeout)
    if(position < 70):          #80 degrees upper limit
        moveTilt = [moveCommand,tiltChannel,chr(70)]
    elif(position > 123):       #5 degrees lower limit
        moveTilt = [moveCommand,tiltChannel,chr(123)]
    else:
        moveTilt = [moveCommand,tiltChannel,chr(position)]
    s.write(moveTilt)
    s.close()

def movePanServo(position):
    ''' Move the vertical servo to a given position '''
    global antennaBear
    print "\tMove Pan: ", float(position)
    s = serial.Serial(str(servoCOM), baudrate = servoBaud, timeout = servoTimeout)
    movePan = [moveCommand,panChannel,chr(255-position)]
    s.write(movePan)
    s.close()

def findMaxRSSI():
    x=127
    y=127
    baseline = getRSSI()
    print "Starting values: pan=%d, tilt=%d, RSSI=%d.".format(x,y,baseline)
    # Pan right until you can't turn any farther, or the signal fades
    while(x < MAXPAN and getRSSI() > baseline):
        movePanServo(x+1)
        print "\tRSSI: ", getRSSI()
    # Pan left until you can't turn any farther, or the signal fades
    while(x > MINPAN and getRSSI() > baseline):
        movePanServo(x-1)
        print "\tRSSI: ", getRSSI()
    # Cancel the last movement (bad idea?)
    movePanServo(x+1)

    # Tilt up until you can't turn any farther, or the signal fades
    while(y < MAXTILT and getRSSI() > baseline):
        moveTiltServo(y+1)
        print "\tRSSI: ", getRSSI()
    # Tilt down until you can't turn any farther, or the signal fades
    while(y > MINTILT and getRSSI() > baseline):
        moveTiltServo(y-1)
        print "\tRSSI: ", getRSSI()
    # Cancel the last movement (bad idea?)
    moveTiltServo(y+1)

def getRSSI():
    #See bookmarks for ideas here
    #https://community.ubnt.com/t5/airOS-Software-Configuration/How-can-i-see-signal-from-command-line/td-p/353927
    #https://community.ubnt.com/t5/airOS-Software-Configuration/How-can-I-remotely-read-RSSI/td-p/249199

if __name__ == '__main__':
    configServos()
    findMaxRSSI()
