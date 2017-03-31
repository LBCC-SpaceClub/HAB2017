import math
import serial
import requests
import time

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
servo_min = 95
servo_max = 159

panChannel = 1
panRange = 360
panAccel = 1
panSpeed = 3
# change the movement speed etc of ubiquity tilt servo
tiltChannel = 0
tiltAccel = 1
tiltSpeed = 1

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
    movement = 5
    delay = 0.8
    global servo_min, servo_max
    with requests.Session() as session:
        session.get('http://192.168.1.20/login.cgi')  # This line has magic sauce
        data={'uri': '','username': 'ubnt','password': 'ubnt'}
        # Use files trick to post using multipart/form-data encoding.
        r = session.post(
            'http://192.168.1.20/login.cgi',
            files={k: (None, v) for k, v in data.items()},
            verify=False
        )
        baseline = getRSSI(session)
        temp = baseline
        while(True):
            #rssi = getRSSI(session)
            #print "Signal={}, rssi={}".format(signal, rssi)
            print "Starting values: pan={}, tilt={}, baseline={}, RSSI={}.".format(x,y,baseline,getRSSI(session))

            '''
            if(x > servo_min):
                x -= movement
                movePanServo(x)
                time.sleep(delay)
                temp = getRSSI(session)            
            ''' 
            # Pan left until you can't turn any farther, or the signal fades
            while(x > servo_min and temp >= baseline):
                x -= movement
                movePanServo(x)
                time.sleep(delay)
                temp = getRSSI(session)
                print "\tDOWN RSSI: ", temp
                if(temp > baseline):
                    baseline = temp            
            '''
            if(x < servo_max):
                x += movement
                movePanServo(x)            
                time.sleep(delay)
                temp = getRSSI(session)    
            '''
            # Pan right until you can't turn any farther, or the signal fades
            while(x < servo_max and temp >= baseline):
                x += movement
                movePanServo(x)
                time.sleep(delay)
                temp = getRSSI(session)                
                print "\tUP RSSI: ", temp    
                if(temp > baseline):
                    baseline = temp
                    
            #time.sleep(0.1)
            baseline -= 0.1
            
def getRSSI(session):
    #https://community.ubnt.com/t5/airOS-Software-Configuration/How-can-i-see-signal-from-command-line/td-p/353927
    #https://community.ubnt.com/t5/airOS-Software-Configuration/How-can-I-remotely-read-RSSI/td-p/249199
    res = session.get('http://192.168.1.20/status.cgi')
    #print "res=", res
    #print res.content
    #signal = res.content.split('\n')[21][11:14]
    rssi = res.content.split('\n')[21][24:26]
    #print "Signal={}, rssi={}\n".format(signal, rssi)
    return int(rssi)

if __name__ == '__main__':
    configServos()   
    findMaxRSSI()
