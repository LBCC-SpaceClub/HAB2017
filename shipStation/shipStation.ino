#include "TinyGPS++.h"

TinyGPSPlus TxGps;
TinyGPSPlus RxGps;

struct GpsPos {
  // A struct to handle both TX and RX Gps locations
  long timeRec;
  double lat;
  char latDir;
  double lon;
  char lonDir;
  long alt;
  
};

GpsPos TXLoc = { 0, 0,'V',0,'V',0 };
GpsPos RXLoc = { 0, 0,'V',0,'V',0 };

void setup() {
  // initialize serial connection and wait for it to be ready
  Serial.begin(9600);
  while(!Serial) ;
  Serial.println("Waiting for input...");
}

void processSerialByte(const byte c){
  // Process new byte of serial input
  // Reads in new TX locations from serial input
  static char cBuffer[256];
  static int index = 0;

  switch(c){
    case '\n': // end of input string
      cBuffer[index] = 0;
      TxGps.encode(cBuffer);
      Serial.print("Tx LAT=");  Serial.print(TxGps.location.lat(), 6);
      Serial.print(", LONG=");  Serial.print(TxGps.location.lng(), 6);
      Serial.print(", ALT=");   Serial.println(TxGps.altitude.meters());
      // eventually, this should probably just feed into TinyGPSPlus's gps2.encode(cBuffer)
      // But for now.. just update the simplified TX location
      // TXLoc = parseGpsString(cBuffer, index);
      index = 0;
      break;
    case '\r': // ignore carriage returns
      break;
    default:   // all other characters are ok
      if(index<255){
        cBuffer[index++] = c;
      }
  }
}

GpsPos parseGpsString(const char * loc, const int sizeOfLoc){
  // Parse a gps string for latitude, latitude direction, longitude, longitude direction, and altitude.
  // Return a GpsPos object
  // Gps strings are expected to look like:
  // $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
  // See: http://aprs.gids.nl/nmea/#rmc
  int index, i;
  GpsPos newPos;
  String buffer = "";
  
  // advance index to the latitude (3751.65), which is in DDMM.MM (decimal degrees)
  i=0;
  while(i<3 && index<sizeOfLoc){
    if(loc[index++] == ',') i++;
  }
  // Convert latitude into a double and store in the gps object
  i=0;
  while(index<sizeOfLoc && loc[index]!=','){
    buffer += loc[index++];
  }
  newPos.lat = buffer.toFloat();
  buffer = "";

  // The next byte should be latitude direction
  newPos.latDir = loc[++index]; // move past the trailing , before, and after
  index += 2; 

  // The next values should become the longitude
  i=0;
  while(index<sizeOfLoc && loc[index]!=','){
    buffer += loc[index++];
  }
  newPos.lon = buffer.toFloat();
  buffer = "";

  // The next byte should be longitude direction
  newPos.lonDir = loc[++index]; // move past the trailing , before, and after
  index += 2;
  
  Serial.print("Lat:"); Serial.print(newPos.lat); Serial.print(","); Serial.print(newPos.latDir); Serial.print("\t");
  Serial.print("Long:"); Serial.print(newPos.lon); Serial.print(","); Serial.print(newPos.lonDir); Serial.print("\t");
//  Serial.print(newPos.alt);
  Serial.println("");
  return { 0, 1,'V',2,'V',3 };
}

void processRXGps(){
  // Update the GPS location of the ship station (lat, long, alt)
}

void loop() {
  // Wait for new serial data, then process it
  if(Serial.available() > 0){
    // processSerialByte(Serial.read());
    TxGps.encode(Serial.read());
  }

  if(TxGps.location.isUpdated()){
    Serial.print("Tx LAT=");  Serial.print(TxGps.location.lat(), 6);
    Serial.print(", LONG=");  Serial.print(TxGps.location.lng(), 6);
    Serial.print(", ALT=");   Serial.println(TxGps.altitude.meters());
  }
  
  // Update RXLoc using GPS shield on arduino

  // Update IMU position and adjust steppers
  
}
