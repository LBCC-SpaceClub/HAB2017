#include "TinyGPS++.h"
#include "SoftwareSerial.h"

#define DEBUGGING true

TinyGPSPlus TxGps;
TinyGPSPlus RxGps;
SoftwareSerial softSerial(8, 7);

void setup() {
  // initialize serial connection and wait for it to be ready
  Serial.begin(115200);
  while(!Serial) ;
  Serial.println("Waiting for input...");
}

void loop() {
  // Wait for new data from the tracking laptop, and process it.
  if(Serial.available() > 0){
    TxGps.encode(Serial.read());
  }

  // Debugging: When the tracker is updated, display the new information
  if(DEBUGGING && TxGps.location.isUpdated()){
    Serial.print("Tx LAT=");  Serial.print(TxGps.location.lat(), 6);
    Serial.print(", LONG=");  Serial.print(TxGps.location.lng(), 6);
    Serial.print(", ALT=");   Serial.print(TxGps.altitude.meters());
    
    Serial.print("\tRx LAT=");  Serial.print(RxGps.location.lat(), 6);
    Serial.print(", LONG=");  Serial.print(RxGps.location.lng(), 6);
    Serial.print(", ALT=");   Serial.println(RxGps.altitude.meters());
  }
  
  // Update RXLoc using GPS shield on arduino
  if(softSerial.available() > 0){
    RxGps.encode(softSerial.read());
  }
  
  // Update IMU position and adjust steppers
  
}
