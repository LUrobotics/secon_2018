

/*
 * Robotics Final Integration Code
 */
 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "Motors.h"
#include "final_Adam.h"

#define tooClose    1
#define tooFar      2
#define good        0
#define tooCounter  1
#define tooClock    2
#define tooLeft     1 
#define tooRight    2

#define testSpeed     70
#define ACTime        3000
#define RampTime      5000


#define BTime         2500
#define Adjust        500
#define BackWallTime  5000
#define Backward      1000
#define ChestTime     1500
#define BackRampTime  9000

/* START & KILL BUTTON STUFF */
const int killButton = SCK;
const int startButton = A5;
bool startProgram = false;

/* IR & 7 SEG STUFF */
int sevSegDisplayNumber = 0;
bool calibrationSignal = true;
Adafruit_7segment matrix = Adafruit_7segment();

/* CAPTAIN'S WHEEL AND ARM TRINKETS' ENABLE PINS */
const int capWheelTrinket = 6;
const int armTrinket = MOSI;

Motors oscar = Motors();

/* NAV STUFF */
int phase = 1;
double sensors[14];
int treasureMap[] = {-1, -1, -1};

double filter1[] = {0.0, 0.0, 0.0};
double filter2[] = {0.0, 0.0, 0.0};
double filter3[] = {0.0, 0.0, 0.0};
double filter4[] = {0.0, 0.0, 0.0};



void setup() {
  Serial.begin(9600);
//  Serial.println("Waiting to start");
  // initialize start and kill buttons
  pinMode(killButton, INPUT_PULLUP); // pin = HIGH when switch open and LOW when switch is pressed
  pinMode(startButton, INPUT_PULLUP);
  pinMode(capWheelTrinket, OUTPUT);
  digitalWrite(capWheelTrinket, LOW);
  pinMode(armTrinket, OUTPUT);
  digitalWrite(armTrinket, LOW);
  // kill button interrupt
  //attachInterrupt(digitalPinToInterrupt(killButton), killFunction, FALLING);
  Wire.begin();

//  // I2C setup for 7-segment display
//  matrix.begin(0x70);
//  // blink colon on 7-segment display until signal received
//  matrix.blinkRate(2);
//  matrix.drawColon(true);
//  matrix.writeDisplay();
//
  muxInit(1);
  muxInit(2);
  muxInit(3);
}

void loop() {
//  // wait until start button is pressed
//  while (digitalRead(startButton) == HIGH) {
//    // receive calibration IR signal
//    readIRSensor();
//  }
//  /* ENTER PROGRAM */
//  
//  // Riley
//  calibrationSignal = false;
//  readIRSensor();

  // perform every 50 ms
//  if(!is50ms()) {
//    return;
//  }

  treasureMap[0] = 1;
  treasureMap[1] = 1;
  treasureMap[2] = 0;
  

  // END WALL PRACTICE


  //Destination A and recenter
  if(treasureMap[0] == 1){
    oscar.StrafeLeft(testSpeed);
    delay(ACTime);
    oscar.Stop();
    delay(100);
    oscar.StrafeRight(testSpeed);
    delay(ACTime-200);
    oscar.Stop();
  }
  else{
    oscar.StrafeRight(testSpeed);
    delay(ACTime);
    oscar.Stop();
    delay(100);
    oscar.StrafeLeft(testSpeed);
    delay(ACTime-200);
    oscar.Stop();
  }

  // center on ramp
  delay(100);
  oscar.DriveBackward(testSpeed);
  delay(Backward);
  oscar.Stop();
  delay(100);
  // drive down ramp
  oscar.DriveForward(testSpeed);
  delay(RampTime);
  oscar.Stop();
  delay(100);

  // Destination B
  if(treasureMap[1] == 1){
    oscar.StrafeLeft(testSpeed);
    delay(BTime);
    oscar.Stop();
    delay(100);
    oscar.StrafeRight(testSpeed);
    delay(Adjust);
    oscar.Stop();
  }
  else{
    oscar.StrafeRight(testSpeed);
    delay(BTime);
    oscar.Stop();
    delay(100);
    oscar.StrafeLeft(testSpeed);
    delay(Adjust);
    oscar.Stop();
  }

  // drive to captains wheel wall
  delay(100);
  oscar.DriveForward(testSpeed);
  delay(BackWallTime);
  oscar.Stop();
  delay(100);
  oscar.DriveBackward(testSpeed);
  delay(250);
  oscar.Stop();

  // centering on wheel
  if(treasureMap[1] == 1) {
      oscar.StrafeLeft(testSpeed);
      delay(1500);
      oscar.StrafeRight(testSpeed);
      delay(ACTime-100);
  } else {
      oscar.StrafeRight(testSpeed);
      delay(1500);
      oscar.StrafeLeft(testSpeed);
      delay(ACTime-100);
      }
  oscar.Stop();
  delay(100);
  oscar.DriveForward(testSpeed);
  delay(400);
  oscar.Stop();
  delay(5000);

//  turn the wheel
//  digitalWrite(capWheelTrinket, HIGH);
//  delay(100);
//  pinMode(capWheelTrinket, INPUT);
//  delay(100);
//  while(digitalRead(capWheelTrinket) == HIGH);

// END WHEEL TURN

  // drive to chest
  oscar.DriveBackward(testSpeed);
  delay(1500);
  oscar.Stop();
  delay(5000);

//  pick up chest
//  digitalWrite(armTrinket, HIGH);
//  delay(100);
//  pinMode(armTrinket, INPUT);
//  while(digitalRead(armTrinket) == HIGH);

//  drive to and up ramp
  oscar.DriveBackward(100);
  delay(BackRampTime);
  oscar.Stop();
  oscar.DriveForward(testSpeed);
  delay(Adjust);
  oscar.Stop();

//  Desination C
  if(treasureMap[2] == 1){
  oscar.StrafeLeft(testSpeed);
  delay(ACTime);
  oscar.Stop();
  }
  else{
  oscar.StrafeRight(testSpeed);
  delay(ACTime);
  oscar.Stop();
  }
  while(1);
}



void centerOnRamp() {
  /*STAY PARALLEL AND WELL SPACED TO BACK WALL*/
  // read back short sensors
  I2CSelect(1,0);
  double backLeftShort = ReadShort();
  I2CSelect(1,-1);
  
  I2CSelect(2,1);
  double backRightShort = ReadShort();
  I2CSelect(2,-1);

  // threshold: 8 mm
  int parallel = isParallel(backLeftShort, backRightShort, 8);
  
  // desired: 120 mm, threshold: 8 mm
  int spacing = hasSpacing(backLeftShort, 120, 8);
  
   if(parallel == tooLeft) {
    oscar.TurnRight(testSpeed);
    return;
  }
  if(parallel == tooRight) {
    oscar.TurnLeft(testSpeed);
    return;
  }
  // else parallel to wall
  if(spacing  == tooClose) {
    oscar.DriveForward(testSpeed);
    return;
  } 
  if(spacing == tooFar) {
    oscar.DriveBackward(testSpeed);
    return;
  } 

  /*CENTER BETWEEN SIDE YELLOW WALLS*/
  // read left side and right side far sensors
  I2CSelect(2,0);
  double leftFar = ReadLong();
  I2CSelect(2,-1);
  
  I2CSelect(1,4);
  double rightFar = ReadLong();
  I2CSelect(1,-1);

  // 8 mm away from center
  int centered = isCentered(leftFar, rightFar, 8);

  if(centered  == tooLeft) {
    oscar.StrafeRight(testSpeed);
    return;
  } 
  if(centered == tooRight) {
    oscar.StrafeLeft(testSpeed);
    return;
  } 

  // else centered and ready to approach ramp
  phase++;
}

void downRampBlind() {
  /* FORWARD UNTIL WE SEE CHEST -- however we (likely) see floor due to ramp angle*/
  // FIXME: lazy
  oscar.DriveForward(testSpeed);
  delay(5000);
  phase++;
}

void seeChest() {
  oscar.DriveForward(testSpeed);
}

void toPressureWall() {
  
}

void seePressurePlate() {
  
}

//void downRamp() {
//
//  // ID1: back close L, ID2: back close R, threshold: 8 mm
//  int parallel = isParallel(4, 5, 8);
//
//  // still close to back wall?
//  if(sensors[4] < 200 && sensors[5] < 200) {
//    if(parallel == tooCounter) {
//      rotateClock();
//      return;
//    }
//    if(parallel == tooClock) {
//      rotateCounter();
//      return;
//    }
//    // else parallel too back wall 
//    moveForward();
//    return;
//  }
//  // else really close to ramp or on ramp
//  // cannot check back wall due to angle of robot
//  
//  // now read floor sensors
//  // ID1: floor L, ID2: floor R, threshold: 8 mm
//  parallel = isParallel(0, 1, 8);
//
//  // ID: floor L, desired: 100 mm, threshold: 25 mm
//  int spacingL = hasSpacing(0, 100, 25);
//
//  if(spacingL == tooFar) {
//    rotateClock();
//    return;
//  }
//  
//  // ID: floor R, desired: 100 mm, threshold: 25 mm
//  int spacingR = hasSpacing(1, 100, 25);
//  if(spacingR == tooFar) {
//    rotateCounter();
//    return;
//  }
//  
//  // else on solid ground 
//
//  // are we close to chest?
//  // ID: front close, desired: 80 mm, threshold: 30 mm
//  int spacingF = hasSpacing(7, 80, 30);
//  if(spacingF == tooClose) {
//    phase++;
//    return;
//  }
//
//  // else on solid ground but not close enough to chest
//  moveForward();
//}
//
//// INSERT: other phase functions
//
int hasSpacing(double reading, float desired, float threshold) {

  float diff = reading - desired;

  if(diff > threshold) {
    return tooFar;
  }
  if(diff < -threshold) {
    return tooClose;
  }
  return good;
}

int isParallel(double left, double right, float threshold) {

  float diff = left - right;

  if(diff > threshold) {
    return tooLeft;
  }
  if(diff < -threshold) {
    return tooRight;
  }
  return good;
}

int isCentered(double left, double right, float threshold) {
  // same as isParallel just meant to match sensors on opposite sides of robot
  float diff = left - right;

  if(diff > threshold) {
    return tooLeft;
  }
  if(diff < -threshold) {
    return tooRight;
  }
  return good;
}

// called when kill button is pressed
void killFunction() {
  //stop moving
  //stop motors
  Serial.println("Kill button pressed"); 
  // do nothing forever 
  while(1);
}

double throughFilter1(double measurement) {
  filter1[0] = filter1[1];
  filter1[1] = filter1[2];
  filter1[2] = measurement;
  return (filter1[0] + filter1[1] + filter1[2]) / 3.0;
}

double throughFilter2(double measurement) {
  filter2[0] = filter2[1];
  filter2[1] = filter2[2];
  filter2[2] = measurement;
  return (filter2[0] + filter2[1] + filter2[2]) / 3.0;
}
