#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define DEBUGGING true

static const uint8_t RXPin = 8, TXPin = 7;
static const uint32_t GPSBaud = 9600;
uint32_t gpsTimer = millis();
uint32_t imuTimer = gpsTimer;
sensors_event_t event;           //Create a new local event instance
uint8_t sys, gyro, accel, mag;   //Create local variables gyro, accel, mag

// Initializes an instance of the BNO055 called bno with an I2C address of 55
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Ultimate GPS shield uses RX and TX on pins 8 and 7
SoftwareSerial ss(RXPin, TXPin);

TinyGPSPlus trackerGPS;
TinyGPSPlus payloadGPS;

void setup()
{
  Serial.begin(115200); // laptop
  ss.begin(GPSBaud);    // local GPS
  while(!Serial){ ; }
  Serial.println(F("Linn-Benton Community College, Eclipse 2017 payload tracker starting up.."));
  
  // Set up BNO055 IMU
  if(!bno.begin()){
    Serial.println(F("Ooops, could not find BNO055... Check your wiring or I2C ADDR!"));
//    while(1);
  } else {
    Serial.println(F("BNO055 IMU detected.."));
  }
//  bno.setExtCrystalUse(true);                     //Use the external clock in the IMU
//  bno.setMode(bno.OPERATION_MODE_NDOF);
}

TinyGPSCustom magneticVariation(trackerGPS, "GPRMC", 10);

void loop()
{
  // If new info is available from the laptop, parse it!
  while(Serial.available() > 0){
    // It's expecting a NMEA string, something like GPRMC:
    // http://www.gpsinformation.org/dale/nmea.htm#RMC
    // $GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n
    payloadGPS.encode(Serial.read());
  }

  // If new info is available from the Adafruit GPS Shield, parse it!
  if (ss.available() > 0){
    trackerGPS.encode(ss.read());
  }

  // Display GPS info about every 2 seconds
  if (millis() - gpsTimer > 2000){
    gpsTimer = millis(); // reset the timer
    
    print_location("local", &trackerGPS);
    print_time("local", &trackerGPS);
    
    print_location("payload", &payloadGPS);
    print_time("payload", &payloadGPS);
    Serial.print("Mag variation: ");
    Serial.println(magneticVariation.value());
  }
  
  // Display IMU info about 10 times per second
  if (millis() - imuTimer > 100){
    imuTimer = millis(); // reset the timer
    // print_local_imu();
  }
}


float findBearing(float tLat, float tLon, float pLat, float pLon)
{
  // Returns bearing from tracker gps to payload gps
  tLat = degToRad(tLat);
  tLon = degToRad(tLon);
  pLat = degToRad(pLat);
  pLon = degToRad(pLon);
  float deltaLon = pLon - tLon;

  float bearing = atan2(
    (sin(deltaLon) * cos(pLat)),
    (cos(tLat) * sin(pLat) - sin(tLat) * cos(pLat) * cos(deltaLon))
  );
  float remainder = round(bearing);
  // Modulus only works on integers
  bearing = int(radToDeg(bearing)+360) % 360;  // stay within 0-360 degrees
  bearing += remainder;
  return bearing;
}


float degToRad(float deg)
{
  // Takes degrees, returns radians
  return deg * M_PI / 180;
}


float radToDeg(float rad)
{
  // Takes radians, returns degrees
  return rad * 180.0 / M_PI;
}


void print_imu(){
  //Read the current calibration values from the IMU
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  //Read the current positional values
  bno.getEvent(&event);
  
  // Using quaternions
  imu::Quaternion q = bno.getQuat();
  q.normalize();
  float temp = q.x();  q.x() = -q.y();  q.y() = temp;
  q.z() = -q.z();
  // Converted back to eulers
  imu::Vector<3> euler = q.toEuler();
  Serial.print(F("[IMU]"));
  Serial.print(-180/M_PI * euler.x());  // heading, nose-right is positive, z-axis points up
  Serial.print(F(","));
  Serial.print(-180/M_PI * euler.y());  // roll, rightwing-up is positive, y-axis points forward
  Serial.print(F(","));
  Serial.print(-180/M_PI * euler.z());  // pitch, nose-down is positive, x-axis points right
  /*
  Serial.print(event.orientation.x,2);
  Serial.print(",");
  Serial.print(event.orientation.y,2);
  Serial.print(",");
  Serial.print(event.orientation.z,2);
  */
  Serial.print(F(","));
  Serial.print(sys);
  Serial.print(F(","));
  Serial.print(gyro);
  Serial.print(F(","));
  Serial.print(accel);
  Serial.print(F(","));
  Serial.println(mag);
}


void print_time(char* desc, TinyGPSPlus* gps){
  // Prints local GPS time to serial
  Serial.print(desc);
  Serial.print(F(": "));
  if (gps->date.isValid() && gps->time.isValid()){
    Serial.print(gps->date.month());
    Serial.print(F("/"));
    Serial.print(gps->date.day());
    Serial.print(F("/"));
    Serial.print(gps->date.year());
    if (gps->time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps->time.hour());
    Serial.print(F(":"));
    if (gps->time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps->time.minute());
    Serial.print(F(":"));
    if (gps->time.second() < 10) Serial.print(F("0"));
    Serial.println(gps->time.second());
  } else {
    Serial.println(F("INVALID"));
  }
}


void print_location(char* desc, TinyGPSPlus* gps){
  // Prints local GPS location to serial
  Serial.print(desc);
  Serial.print(F(": "));
  if (gps->location.isValid()){
    Serial.print(gps->location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps->location.lng(), 6);
    Serial.print(F(","));
    Serial.println(gps->altitude.meters());
  }
  else
  {
    Serial.println(F("INVALID"));
  }
}
