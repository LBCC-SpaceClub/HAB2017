#include <AccelStepper.h>
//#include <MultiStepper.h>
#include <math.h>

/* Written for ST6600 stepper driver
 *  PUL+  5v
 *  PUL-  Arduino pin 9/11
 *  DIR+  5v
 *  DIR-  Arduino pin 8/10
 *  DC+   24v power supply
 *  DC-   Gnd on power supply & Gnd on Arduino
 */

// Each step is 0.12 degrees, or 0.002094395 radians
#define STEPANGLE 0.002094395
#define PI 3.1415926535897932384626433832795

AccelStepper xAxis(1,12,11);
AccelStepper yAxis(1,10,9);
//MultiStepper steppers;

float angle = 0;          // Radians, used to find x,y on a circle
long centerX = 0;         // May be useful later
long centerY = 0;
long radius = 20000;

void setup()
{
  Serial.begin(115200);
  
  // Adjust these values after seeing it in action.
  // 1/4  step: 1600, 800
  // 1/16 step: 10000, 5000
  xAxis.setMaxSpeed(10000);
  xAxis.setAcceleration(5000);
  yAxis.setMaxSpeed(10000);
  yAxis.setAcceleration(5000);
}


void moveRandomly(){
  long randNum, xPos, yPos, curTime, endTime;
  while(1){
    // Random numbers from -25,000 to +25,000
    xPos = random(30000) - 15000;
    xAxis.moveTo(xPos);
    
    yPos = random(20000) - 10000;
    yAxis.moveTo(yPos);
    
    curTime = millis();
    endTime = curTime + random(10000);
    
    Serial.print("Moving to ");
    Serial.print(xPos);
    Serial.print(", ");
    Serial.print(yPos);
    Serial.print(", for ");
    Serial.print(endTime - curTime);
    Serial.println(" millis.");
    
    while(millis() < endTime){
      xAxis.run();
      yAxis.run();
    }
  }
}

void loop()
{
   moveRandomly();
}
