#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <AccelStepper.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <utility/imumaths.h>
#include <Wire.h>

#define DEBUGGING true
#define GPSBAUD 9600

static const uint8_t RXPin = 8, TXPin = 7;
uint32_t gpsTimer = millis();
uint32_t imuTimer = gpsTimer;
sensors_event_t event;           //Create a new local event instance for IMU
uint8_t sys, gyro, accel, mag;
double azimuth_deg, elevation_deg, distance_meters, delta_altitude;
double azimuth_steps, elevation_steps;

// Initializes the BNO055 using I2C address 55
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Ultimate GPS shield uses RX and TX on pins 8 and 7
SoftwareSerial ss(RXPin, TXPin);

TinyGPSPlus trackerGPS;
TinyGPSPlus payloadGPS;
TinyGPSCustom magneticVariation(trackerGPS, "GPGGA", 13);

AccelStepper xAxis(1,12,11);
AccelStepper yAxis(1,10,9);


void setup()
{
  Serial.begin(115200); // laptop
  while(!Serial);
  Serial.println(F("Linn-Benton Community College, Eclipse 2017 stepper controller starting up.."));

  // Start GPS and tell it to send GPGGA and GPRMC strings
  ss.begin(GPSBAUD);    // local GPS
  //ss.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set up BNO055 IMU
  if(!bno.begin()){
    Serial.println(F("Ooops, could not find BNO055... Check your wiring or I2C ADDR!"));
  } else {
    Serial.println(F("BNO055 IMU detected.."));
  }
  delay(100); // give BNO055 time to start up
  bno.setExtCrystalUse(true);                     //Use the external clock in the IMU
//  bno.setMode(bno.OPERATION_MODE_NDOF);

  // Set stepper motor acceleration and top speeds
  xAxis.setMaxSpeed(8000);
  xAxis.setAcceleration(4000);
  yAxis.setMaxSpeed(8000);
  yAxis.setAcceleration(4000);
  // Temporary
  balance();
}


void loop()
{
  // If new info is available from the laptop, parse it!
  while(Serial.available() > 0){
    // It's expecting a NMEA string, something like GPGGA:
    // http://www.gpsinformation.org/dale/nmea.htm#RMC
    // '$GPGGA,123519,4433.109,N,12314.066,W,1,08,0.9,545.4,M,46.9,M,,*5a'
    payloadGPS.encode(Serial.read());
  }

  // If new info is available from the Adafruit GPS Shield, parse it!
  if(ss.available() > 0){
    trackerGPS.encode(ss.read());
  }

  if(payloadGPS.location.isUpdated() || trackerGPS.location.isUpdated()){
    // Update target solution every time new GPS info is available
    get_tracking_solution();
    // Update solution based on IMU
    updateMotors(azimuth_deg, elevation_deg);
  }

  xAxis.run();
  yAxis.run();

  // Display GPS info about every 2 seconds
  if(millis() - gpsTimer > 2000){
    gpsTimer = millis(); // reset the timer

    print_location("[TGPS]", &trackerGPS);
    //print_location("[PAYLOAD]", &payloadGPS);
    print_time("[TIME]", &trackerGPS);
    print_solution("[SOL]");
    Serial.println();
  }

  // Display IMU info about 10 times per second
  if(millis() - imuTimer > 100){
    imuTimer = millis(); // reset the timer
    // print_local_imu();
  }
}


void updateMotors(double aziDegs, double eleDegs){
  if(isnan(aziDegs) || isnan(eleDegs)){
    Serial.println("Moving to 0, 0");
    xAxis.moveTo(0);
    yAxis.moveTo(0);
  } else {
    long a = degreesToSteps(aziDegs);
    long e = -degreesToSteps(eleDegs);
//    Serial.print(F("yaw="));
//    Serial.print(aziDegs);
//    Serial.print(F(", "));
//    Serial.print(a);
//    Serial.print(F(", pitch="));
//    Serial.print(eleDegs);
//    Serial.print(F(", "));
//    Serial.print(e);
//    Serial.print(F(", calibrations: "));
//    Serial.print(sys);
//    Serial.print(F(", "));
//    Serial.print(gyro);
//    Serial.print(F(", "));
//    Serial.print(accel);
//    Serial.print(F(", "));
//    Serial.println(mag);
    xAxis.moveTo(a);
    yAxis.moveTo(e);

  }
}


void balance(){
  while(1){
    // Read IMU 10 times per second
    if(millis() - imuTimer > 250){
      imuTimer = millis(); // reset the timer

      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      long x = map(euler.x(), 0, 360, 0, 45900);
      long y = map(euler.y(), 0, 360, 0, 45900);
      long z = map(euler.z(), 0, 360, 0, 45900);
//      Serial.print("x=");     Serial.print(euler.x());      Serial.print(" -> ");     Serial.println(x);
//      Serial.print("y=");     Serial.print(euler.y());      Serial.print(" -> ");     Serial.println(y);
//      Serial.print("z=");     Serial.print(euler.z());      Serial.print(" -> ");     Serial.println(z);
//      Serial.println("------------------");
      xAxis.moveTo(y);
      yAxis.moveTo(-z);
    }
  
    xAxis.run();
    yAxis.run();
  }
}


long degreesToSteps(double deg){
  // Takes degrees, returns steps (assuming 16*3060 = 48960 total steps)
  // deg / 360 = steps / (microsteps * motor steps)
  // 3000/0.12*15.3 = 45900
  // deg * 45900 / 360
  return round(deg * 127.5);
}

double radsToDegrees(double rad){
  // Takes radians, returns degrees
  return rad * 180 / M_PI;
}


void get_tracking_solution(){
  // Update the tracking solution variables when new gps information is available

  // Distance across a sphere from tracker to payload
  distance_meters = TinyGPSPlus::distanceBetween(
    trackerGPS.location.lat(),
    trackerGPS.location.lng(),
    payloadGPS.location.lat(),
    payloadGPS.location.lng()
  );

  // Altitude from tracker up to payload
  delta_altitude = payloadGPS.altitude.meters() - trackerGPS.altitude.meters();

  // Horizontal degrees from true north
  azimuth_deg = TinyGPSPlus::courseTo(
    trackerGPS.location.lat(),
    trackerGPS.location.lng(),
    payloadGPS.location.lat(),
    payloadGPS.location.lng()
  );

  // Degrees from vertical at tracker, down to payload
  elevation_deg = atan2(delta_altitude, distance_meters) * 57296 / 1000;
}


void print_solution(char* desc){
  // Order is: azimuth, elevation, distance, altitude,
  if(trackerGPS.location.isValid() && payloadGPS.location.isValid()){
    Serial.print(desc);
    Serial.print(azimuth_deg);
    Serial.print(F(","));
    Serial.print(elevation_deg);
    Serial.print(F(","));
    Serial.print(distance_meters);
    Serial.print(F(","));
    Serial.println(magneticVariation.value());
  } else {
    Serial.print(F("INV: "));
    Serial.print(trackerGPS.location.isValid()==1?"tracker ok":"tracker BAD");
    Serial.print(F(","));
    Serial.println(payloadGPS.location.isValid()==1?"payload ok":"payload BAD");
  }
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
  if (gps->date.isValid() && gps->time.isValid()){
    Serial.print(gps->date.month());
    Serial.print(F("/"));
    Serial.print(gps->date.day());
    Serial.print(F("/"));
    Serial.print(gps->date.year());
    Serial.print(F(","));
    if (gps->time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps->time.hour());
    Serial.print(F(":"));
    if (gps->time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps->time.minute());
    Serial.print(F(":"));
    if (gps->time.second() < 10) Serial.print(F("0"));
    Serial.print(gps->time.second());
    Serial.println(" UTC.");
  } else {
    Serial.println(F("INVALID TIME"));
  }
}


void print_location(char* desc, TinyGPSPlus* gps){
  // Prints local GPS location to serial
  Serial.print(desc);
  if (gps->location.isValid()){
    Serial.print(gps->location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps->location.lng(), 6);
    Serial.print(F(","));
    Serial.println(gps->altitude.meters());
  }
  else
  {
    Serial.println(F("INVALID LOCATION"));
  }
}
