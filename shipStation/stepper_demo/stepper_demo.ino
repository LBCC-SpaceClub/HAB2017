#include <AccelStepper.h>
#include <MultiStepper.h>
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

AccelStepper xAxis(1,8,9);
AccelStepper yAxis(1,10,11);
MultiStepper steppers;

long positions[2];        // x, y
float angle = 0;          // Radians, used to find x,y on a circle
long centerX = 0;         // May be useful later
long centerY = 0;
long radius = 100;

void setup()
{
  // Initialize pins
  pinMode(8, OUTPUT);     // x direction pin
  pinMode(9, OUTPUT);     // x step pin
  pinMode(10, OUTPUT);    // y direction pin
  pinMode(11, OUTPUT);    // y step pin

  Serial.begin(115200);
  
  // Adjust these values after seeing it in action.
  xAxis.setMaxSpeed(300);
  xAxis.setAcceleration(100);
  yAxis.setMaxSpeed(300);
  yAxis.setAcceleration(100);

  // Add individual steppers to the multistepper object
  steppers.addStepper(xAxis);
  steppers.addStepper(yAxis);
}

void updatePos(float degs)
{
    // Moves to a point on a circle, based on radius and angle
    // Blocks until finished
    float rads = degs * PI / 180;
    positions[0] = radius*cos(rads)+centerX; // x
    positions[1] = radius*sin(rads)+centerY; // y
    Serial.print("Deg = ");
    Serial.print(degs);
    Serial.print(", ");
    Serial.print(positions[0]);
    Serial.print(", ");
    Serial.print(positions[1]);
    Serial.println(".");
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();  
}

void loop()
{
  // Spiral outwards
  for(int degs=0; degs<360; degs++){
    updatePos(degs);
    radius += 10;
  }

  // Spiral inwards
  for(int degs=360; degs>0; degs--){
    updatePos(degs);
    radius -= 10;
  }
}
