#include "TinyGPS++.h"
#include <SoftwareSerial.h>

#define DEBUGGING true

static const int RXPin = 8, TXPin = 7;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus TxGps;
TinyGPSPlus RxGps;
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

void displayInfo(){
  // txLat, txLong, txAlt, rxlat, rxLong, rxAlt, bearing from rx to tx.
  Serial.print(TxGps.location.lat(), 6); Serial.print(",");
  Serial.print(TxGps.location.lng(), 6); Serial.print(",");
  Serial.print(TxGps.altitude.meters()); Serial.print(",");
  Serial.print(RxGps.location.lat(), 6); Serial.print(",");
  Serial.print(RxGps.location.lng(), 6); Serial.print(",");
  Serial.print(RxGps.altitude.meters()); Serial.print(",");
  double courseTo = TinyGPSPlus::courseTo(
    TxGps.location.lat(), TxGps.location.lng(),
    RxGps.location.lat(), RxGps.location.lng()
  );
  Serial.print(courseTo); Serial.print(",");
  Serial.println(TinyGPSPlus::cardinal(courseTo));
}

void loop() {
  // Wait for new data from the tracking laptop, and process it.
  while(Serial.available() > 0){
    TxGps.encode(Serial.read());
  }
  
  // Update RXLoc using GPS shield on arduino
  while(ss.available() > 0){
    RxGps.encode(ss.read());
  }
    
  // Update IMU position and adjust steppers
  
  // Debugging: When the tracker is updated, display the new information
  if(DEBUGGING && (TxGps.location.isUpdated() || RxGps.location.isUpdated()) ){
    displayInfo();
  }
}
