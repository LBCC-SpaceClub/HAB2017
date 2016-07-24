#include <Servo.h>
Servo myservo;
int pos = 0;

void setup() {
  myservo.attach(9);
  myservo.write(45);  // Start out in a good position
  delay(5000);        // Wait as long as you like. (milliseconds)
}

void loop() {
  myservo.write(130)  // End in a release position
  while(true);        // Do nothing, forever.
}
