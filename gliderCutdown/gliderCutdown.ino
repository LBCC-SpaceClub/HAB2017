// Change this number to change the delay before cutdown triggers
int delayMinutes = 100;

void waitOneMin(){
  // Waits one minute
  delay(60000);
}

void setup(){
  // Connect pin A0 to the cutdown motor
  pinMode(A0, OUTPUT);
}

void loop(){
  // Wait x minutes, then cut!
  while(delayMinutes>0){
    waitOneMin();
    delayMinutes--;
  }
  // Done waiting, time to run the cutdown!
  digitalWrite(A0, HIGH);
  // Run cutdown for 1 minute
  waitOneMin();
  // Then turn it off
  digitalWrite(A0, LOW);
  // Stop doing anything.
  while(true);
}
