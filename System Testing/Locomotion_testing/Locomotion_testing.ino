#include <Motors.h>

#define testSpeed 30.0

Motors oscar = Motors();

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing Motor Test");
}

void loop(){
  oscar.DriveForward(testSpeed);
  delay(1000);
  oscar.StrafeRight(testSpeed);
  delay(1000);
  oscar.DriveBackward(testSpeed);
  delay(1000);
  oscar.StrafeLeft(testSpeed);
  delay(1000);
}

