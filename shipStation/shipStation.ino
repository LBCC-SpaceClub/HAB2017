#include "TinyGPS++.h"
#include <SoftwareSerial.h>

#define DEBUGGING true

static const int RXPin = 8, TXPin = 7;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus PayloadGps;
TinyGPSPlus StationGps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  // initialize serial connection and wait for it to be ready
  Serial.begin(9600);
  Serial.flush();
  while(!Serial) ;

  // initialize software serial connection
  ss.begin(GPSBaud);
  
  Serial.println("Arduino booted up and ready for input..");
}

double HaverSine(double lat1, double lon1, double lat2, double lon2){
  // Returns distance in km between two gps coordinates
  // Credit to https://github.com/RobTillaart/Arduino
  const double ToRad = PI / 180.0;
  const double R = 6371;   // radius earth in Km

  double dLat = (lat2-lat1) * ToRad;
  double dLon = (lon2-lon1) * ToRad;

  double a = sin(dLat/2) * sin(dLat/2) +
    cos(lat1 * ToRad) * cos(lat2 * ToRad) *
    sin(dLon/2) * sin(dLon/2);

  double c = 2 * atan2(sqrt(a), sqrt(1-a));

  double d = R * c;
  return d; // returns km!
}

void displayInfo(){
  Serial.print(PayloadGps.location.lat(), 6); Serial.print(",");
  Serial.print(PayloadGps.location.lng(), 6); Serial.print(",");
  Serial.print(PayloadGps.altitude.meters()); Serial.print(",");
  Serial.print(StationGps.location.lat(), 6); Serial.print(",");
  Serial.print(StationGps.location.lng(), 6); Serial.print(",");
  Serial.print(StationGps.altitude.meters()); Serial.print(",");
  
  // courseTo() is bearing relative to true north
  double courseTo = TinyGPSPlus::courseTo(
    StationGps.location.lat(), StationGps.location.lng(),
    PayloadGps.location.lat(), PayloadGps.location.lng()
  );
  Serial.print(courseTo); Serial.print(",");
  Serial.print(TinyGPSPlus::cardinal(courseTo)); Serial.print(",");
  
  Serial.print(courseTo); Serial.print(",");
  Serial.println(HaverSine(
    StationGps.location.lat(), StationGps.location.lng(),
    PayloadGps.location.lat(), PayloadGps.location.lng())
  );
}

void loop() {
  // Wait for new data from the tracking laptop, and process it.
  while(Serial.available() > 0){
    PayloadGps.encode(Serial.read());
  }
  
  // Update RXLoc using GPS shield on arduino
  while(ss.available() > 0){
    StationGps.encode(ss.read());
  }
    
  // Update IMU position and adjust steppers
  
  // Debugging: When the tracker is updated, display the new information
  if(DEBUGGING && (PayloadGps.location.isUpdated() || StationGps.location.isUpdated()) ){
    displayInfo();
  }
}
