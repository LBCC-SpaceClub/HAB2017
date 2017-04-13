/*
 * Author: Dylan Trafford (EE/CpE), Trevor Gahl (CpE), Gabe Gordon (MSGC MAP Student). Adapted from example code from Adafruit.com
 * Modified for Linn-Benton Community College by Levi Willmeth
 * Developed for use by MSGC BOREALIS Program
 * Purpose: To transmit data from a GPS and IMU unit to a computer for ground station positional data.
 * Note: Sends comma seperated data lead by a '~'. When the recieving computer sees a tilda, it knows it is the beginning of the line.
 */

//Included Libraries (some libraries are imported even though they are included in the base packages)
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <avr/sleep.h>

//Defines (string replace constants)
#define BNO055_SAMPLERATE_DELAY_MS (100)                    //Set the delay between reading from the IMU, unused in this version
#define GPSECHO  false                                      //Display GPS data as read from the GPS in the Serial Monitor (dont do for actual use but good for debugging)

//Instance Initializations
Adafruit_BNO055 bno = Adafruit_BNO055(55);                  //Initializes an instance of the BNO055 called bno with an I2C address of 55
SoftwareSerial mySerial(8, 7);                              //Initializes an instance of SoftwareSerial called mySerial with RX and TX on pins 8 and 7
Adafruit_GPS GPS(&mySerial);                                //Initializes an instance of Adafruit_GPS called GPS using the mySerial instance

//Global Intializations
boolean usingInterrupt = true;                              //Use an interrupt to parce GPS data (preferred to be true)
boolean calibrated = true;
int initial = false;

void setup() {
  //Launches a serial connection with a 115200 baud rate
  Serial.begin(115200);
  while(!Serial){ ; }
  Serial.println("Linn-Benton Community College, Eclipse 2017 payload tracker starting up..");
  //Launches the IMU. It returns a true value if it successfully launches.
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } else {
    Serial.println("BNO055 IMU detected..");
  }
  GPS.begin(9600);                                        //Launches a software serial connection to the GPS at a baud rate of 9600
  delay(500);                                               //Wait for 0.5s
  bno.setExtCrystalUse(true);                               //Use the external clock in the IMU (true for better accuracy)
  bno.setMode(bno.OPERATION_MODE_NDOF);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);             //String formatting on the GPS
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                //GPS packet dump rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(usingInterrupt);                             //Set to use or not use the interrupt for GPS parcing
  delay(100);                                               //Wait for 0.1s
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  // Set up the interrupt, intended to be run once on startup
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
sensors_event_t event;                                    //Create a new local event instance.... called event
uint8_t sys, gyro, accel, mag;                            //Create local variables gyro, accel, mag

void loop() {
  //Read the calibration values from the IMU
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  bno.getEvent(&event);

  //If new GPS data is ready, parse it!
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("Time: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print(" on ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC);Serial.print("/20");
    Serial.println(GPS.year, DEC);
    if(GPS.fix){
      // GPS data on one line
      Serial.print("[GPS]");
      Serial.print(GPS.latitudeDegrees,7);
      Serial.print(",");
      Serial.print(GPS.longitudeDegrees,7);
      Serial.print(",");
      Serial.println((int)(GPS.altitude));
      // Serial.println((int)(GPS.altitude * 3.28084)); // converted m to feet
      // Followed by IMU data
      Serial.print("[IMU]");
      Serial.print(event.orientation.x,2);
      Serial.print(",");
      Serial.print(sys);
      Serial.print(",");
      Serial.print(gyro);
      Serial.print(",");
      Serial.print(accel);
      Serial.print(",");
      Serial.println(mag);
    }else{
      Serial.print("No GPS fix!  GPS Quality: ");
      Serial.println(GPS.fixquality);
      Serial.print(GPS.latitudeDegrees,7);
    }
  }
}
