#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <AccelStepper.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <EEPROM.h>

#define DEBUGGING true
#define GPSBAUD 9600

// GPS variables
static const uint8_t RXPin = 8, TXPin = 7;
uint32_t gpsTimer = millis();
double azimuth_deg, elevation_deg, distance_meters, delta_altitude;

// IMU variables
uint32_t imuTimer = millis();
sensor_t sensor;
int eeAddress = 0;  // Any valid address in EEPROM
long bnoID = 42;    // Any unique ID for this IMU
bool recentlySaved = false;
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
    bno.setExtCrystalUse(true);                     //Use the external clock in the IMU
    bno.setMode(bno.OPERATION_MODE_NDOF);
    if(restore_imu_offsets()){
      Serial.println(F("Loaded IMU offsets from EEPROM."));
    } else {
      Serial.println(F("Could not load offsets from EEPROM."));
    }
  }
  
  // Set stepper motor acceleration and top speeds
  xAxis.setMaxSpeed(7500);
  xAxis.setAcceleration(4000);
  yAxis.setMaxSpeed(7500);
  yAxis.setAcceleration(4000);
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
    apply_IMU();
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
    print_imu();
    Serial.println();
    if(!recentlySaved && bno.isFullyCalibrated()){
      recentlySaved = backup_imu_offsets();
    }
  }

  // Display IMU info about 5 times per second
  if(millis() - imuTimer > 200){
    imuTimer = millis(); // reset the timer
  }
}


boolean restore_imu_offsets(){
  // Check EEPROM to see if our IMU offsets have been saved
  EEPROM.get(eeAddress, bnoID);
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id){
    Serial.println("No IMU Calibration in EEPROM.");
    return false;
  } else {
    Serial.println("Loading BNO055 calibration data from EEPROM.");
    adafruit_bno055_offsets_t calibrationData;
    int temp = eeAddress + sizeof(long);
    EEPROM.get(temp, calibrationData);
    bno.setSensorOffsets(calibrationData);
    return true;
  }
}

boolean backup_imu_offsets(){
  // Back up IMU offsets into EEPROM
  bno.getSensor(&sensor);
  if(bno.isFullyCalibrated()){
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    bnoID = sensor.sensor_id;
    // Write our IMU ID out to EEPROM
    EEPROM.put(eeAddress, bnoID);
    // Write the current (calibrated) IMU offsets out to EEPROM
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    return true;
  }
  return false;
}


void updateMotors(double aziDegs, double eleDegs){
  long azimuth_steps = map(aziDegs, 0, 360, 0, 45900);
  long elevation_steps = map(eleDegs, 0, 360, 0, 45900);
  xAxis.moveTo(azimuth_steps);
  yAxis.moveTo(elevation_steps);
}


void apply_IMU(){   }


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
    Serial.print(desc);
    Serial.print(F("INVALID GPS: "));
    Serial.print(trackerGPS.location.isValid()==1?"tracker ok":"tracker BAD");
    Serial.print(F(", "));
    Serial.println(payloadGPS.location.isValid()==1?"payload ok":"payload BAD");
  }
}


void print_imu(){
  //Read the current calibration values from the IMU
  //sensors_event_t event;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(F("[IMU]"));
  Serial.print(euler.x());
  Serial.print(F(","));
  Serial.print(euler.y());
  Serial.print(F(","));
  Serial.print(euler.z());

  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F(","));
  Serial.print(sys, DEC);
  Serial.print(F(","));
  Serial.print(gyro, DEC);
  Serial.print(F(","));
  Serial.print(accel, DEC);
  Serial.print(F(","));
  Serial.println(mag, DEC);
}


void print_time(char* desc, TinyGPSPlus* gps){
  // Prints local GPS time to serial
  if (gps->date.isValid() && gps->time.isValid()){
    Serial.print(desc);
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
  }
}


void print_location(char* desc, TinyGPSPlus* gps){
  // Prints local GPS location to serial
  if (gps->location.isValid()){
    Serial.print(desc);
    Serial.print(gps->location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps->location.lng(), 6);
    Serial.print(F(","));
    Serial.println(gps->altitude.meters());
  }
}
