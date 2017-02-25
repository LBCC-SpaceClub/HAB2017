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

double findElevation(double sAlt, double pAlt, double dist){
  // Returns the elevation (vertical angle) from station to payload, in degrees for consistency
  static const double R = 6371;    // avg radius of earth in Km
  sAlt = sAlt+R;                   // adjust to include the surface of the earth
  pAlt = pAlt+R;                   // km
  // interior angle, comes from arclength
  const double r = dist/R;         // rad
  // Straight distance from payload to station
  const double D = pow(sAlt,2)+pow(pAlt,2)-2*sAlt*pAlt*cos(r);
  // Angle from payload down to station
  double pAngle = asin(pAlt/D*sin(r));
    
  // Debugging:
  Serial.print('>');
  Serial.print(sAlt);   Serial.print(',');
  Serial.print(pAlt);   Serial.print(',');
  Serial.print(D);      Serial.print(',');
  Serial.print(r);      Serial.print(',');
  Serial.print(pAngle); Serial.print(',');
  Serial.print(PI-r-pAngle); Serial.print(','); 
  
  // Convert to degrees (ugh)
//  pAngle *= 180.0/PI;
//  r *= 180.0/PI;

  Serial.print('<');
  // Interior angles sum to 180 degrees
  return double(PI-pAngle-r);
}

void displayInfo(){
  Serial.print(PayloadGps.location.lat(), 6); Serial.print(',');
  Serial.print(PayloadGps.location.lng(), 6); Serial.print(',');
  Serial.print(PayloadGps.altitude.meters()); Serial.print(',');
  Serial.print(StationGps.location.lat(), 6); Serial.print(',');
  Serial.print(StationGps.location.lng(), 6); Serial.print(',');
  Serial.print(StationGps.altitude.meters()); Serial.print(',');
  
  // courseTo() is bearing relative to true north
  double courseTo = TinyGPSPlus::courseTo(
    StationGps.location.lat(), StationGps.location.lng(),
    PayloadGps.location.lat(), PayloadGps.location.lng()
  );
  Serial.print(courseTo); Serial.print(',');
  Serial.print(TinyGPSPlus::cardinal(courseTo)); Serial.print(',');

//  double surfaceDistance = HaverSine(
//    StationGps.location.lat(), StationGps.location.lng(),
//    PayloadGps.location.lat(), PayloadGps.location.lng()
//  );
//  Serial.print(surfaceDistance); Serial.print(',');
  double surfaceDistance = TinyGPSPlus::distanceBetween(
    StationGps.location.lat(), StationGps.location.lng(),
    PayloadGps.location.lat(), PayloadGps.location.lng()
  );
  Serial.print(surfaceDistance); Serial.print(',');

  double vertAngle = findElevation(
    StationGps.altitude.kilometers(),
    PayloadGps.altitude.kilometers(),
    surfaceDistance
  );
  Serial.print(vertAngle); Serial.print(',');

  // demo comparison:
  double relAlt = (PayloadGps.altitude.meters()-StationGps.altitude.meters())/1000;
//  Serial.print(relAlt); Serial.print('/');
//  Serial.print(surfaceDistance); Serial.print('=');
  Serial.println( atan2(relAlt, surfaceDistance)*180.0/PI );
//  Serial.println( atan2(940,1010) );
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
