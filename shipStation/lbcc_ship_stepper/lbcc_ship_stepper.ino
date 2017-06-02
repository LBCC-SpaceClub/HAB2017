#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define DEBUGGING true
#define INPUT_BUF_SIZE 128

// Initializes an instance of the BNO055 called bno with an I2C address of 55
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Ultimate GPS shield uses RX and TX on pins 8 and 7
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
Adafruit_GPS payloadGPS = new Adafruit_GPS();

uint32_t gpsTimer = millis();
uint32_t imuTimer = gpsTimer;
sensors_event_t event;           //Create a new local event instance
uint8_t sys, gyro, accel, mag;   //Create local variables gyro, accel, mag
char inputBuffer[INPUT_BUF_SIZE];


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

int parseSerial()
{
  char c;
  uint8_t i = 0;
  // Wait for a leading tilde
  while(Serial.available()){
    if(Serial.read() == '~') break;
  }
  // Process the rest of the line
  while(Serial.available() && i<INPUT_BUF_SIZE-1){
    c = Serial.read();
    
  }
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

void setup()
{
  Serial.begin(115200);
  while(!Serial){ ; }
  Serial.println("Linn-Benton Community College, Eclipse 2017 payload tracker starting up..");
  
  // Set up BNO055 IMU
  if(!bno.begin()){
    Serial.println("Ooops, could not find BNO055... Check your wiring or I2C ADDR!");
//    while(1);
  } else {
    Serial.println("BNO055 IMU detected..");
  }
//  bno.setExtCrystalUse(true);                     //Use the external clock in the IMU
//  bno.setMode(bno.OPERATION_MODE_NDOF);

  // Set up Adafruit Ultimate GPS Shield
  GPS.begin(9600);
  // Set GPS to update position once per second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  // Specify GPS output style
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PGCMD_ANTENNA);
  delay(100);
}

void loop()
{
  //If new GPS data is ready, parse it!
  if (GPS.newNMEAreceived()){
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  payloadGPS.parse("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
  Serial.print("Payload GPS: ");
  Serial.print(payloadGPS.latitudeDegrees,7);
  Serial.print(',');
  Serial.print(payloadGPS.longitudeDegrees,7);
  Serial.print(',');
  Serial.println((int)(payloadGPS.altitude));

  // Display GPS info about every 2 seconds
  if (millis() - gpsTimer > 2000){
    gpsTimer = millis(); // reset the timer
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
      Serial.print(',');
      Serial.print(GPS.longitudeDegrees,7);
      Serial.print(',');
      Serial.println((int)(GPS.altitude));
      // Serial.println((int)(GPS.altitude * 3.28084)); // converted m to feet
    }else{
      Serial.print("No GPS fix!  GPS Quality: ");
      Serial.println(GPS.fixquality);
    }
  }
//  // Display IMU info about 10 times per second
//  if (millis() - imuTimer > 100){
//    imuTimer = millis(); // reset the timer
//    //Read the current calibration values from the IMU
//    bno.getCalibration(&sys, &gyro, &accel, &mag);
//    //Read the current positional values
//    bno.getEvent(&event);
//
//    // Using quaternions
//    imu::Quaternion q = bno.getQuat();
//    q.normalize();
//    float temp = q.x();  q.x() = -q.y();  q.y() = temp;
//    q.z() = -q.z();
//    // Converted back to eulers
//    imu::Vector<3> euler = q.toEuler();
//    Serial.print("[IMU]");
//    Serial.print(-180/M_PI * euler.x());  // heading, nose-right is positive, z-axis points up
//    Serial.print(',');
//    Serial.print(-180/M_PI * euler.y());  // roll, rightwing-up is positive, y-axis points forward
//    Serial.print(',');
//    Serial.print(-180/M_PI * euler.z());  // pitch, nose-down is positive, x-axis points right
//    /*
//    Serial.print(event.orientation.x,2);
//    Serial.print(",");
//    Serial.print(event.orientation.y,2);
//    Serial.print(",");
//    Serial.print(event.orientation.z,2);
//    */
//    Serial.print(',');
//    Serial.print(sys);
//    Serial.print(',');
//    Serial.print(gyro);
//    Serial.print(',');
//    Serial.print(accel);
//    Serial.print(',');
//    Serial.println(mag);
//  }
}
