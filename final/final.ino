/*
 * Robotics Final Integration Code
 */
 
#include <IRLibDecodeBase.h> 
#include <IRLib_HashRaw.h>  //Must be last protocol
#include <IRLibCombo.h>     // After all protocols, include this
#include <IRLibRecvLoop.h> 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_VL53L0X.h" // https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code
#include "Adafruit_VL6180X.h" // https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/wiring-and-test
#include <Motors.h>

#define TCAADDR1 0x71
#define TCAADDR2 0x72
#define TCAADDR3 0x73

#define testSpeed 30.0


#define tooClose    1
#define tooFar      2
#define good        0
#define tooCounter  1
#define tooClock    2
#define tooLeft     1
#define tooRight    2

/* START & KILL BUTTON STUFF */
const int killButton = SCK;
const int startButton = A5;
bool startProgram = false;

/* IR & 7 SEG STUFF */
const int irRecvPin = A4;
IRdecode myDecoder;
IRrecv myReceiver(irRecvPin);
int sevSegDisplayNumber = 0;
bool calibrationSignal = true;
Adafruit_7segment matrix = Adafruit_7segment();

/* NAV STUFF */
int phase = 1;
double sensors[14];
int treasureMap[] = {-1, -1, -1};

double filter1[3];
double filter2[3];
double filter3[3];

// objects
Adafruit_VL6180X shortRange = Adafruit_VL6180X();
Adafruit_VL53L0X longRange = Adafruit_VL53L0X();
Motors oscar = Motors();

void setup() {
  Serial.begin(9600);
//  while(!Serial);
  Serial.println("Waiting to start");
  // initialize start and kill buttons
  pinMode(killButton, INPUT_PULLUP); // pin = HIGH when switch open and LOW when switch is pressed
  pinMode(startButton, INPUT_PULLUP);
  
  
  // kill button interrupt
//  attachInterrupt(digitalPinToInterrupt(killButton), killFunction, FALLING);
  // IR Receiver
  Serial.println("Enabling IRin");
  myReceiver.enableIRIn(); // Start the receiver
  Serial.println("Enabled IRin");
  // I2C setup for 7-segment display
  matrix.begin(0x70);
  // blink colon on 7-segment display until signal received
  matrix.blinkRate(2);
  matrix.drawColon(true);
  matrix.writeDisplay();

  setupLidar(); 
  
}

void loop() {

//  while(true) {
//    lidarReadings();
//  }
  //rileyMaps();

  readIRSensor();
  myReceiver.disableIRIn();
  
  treasureMap[0] = 1;



  // HARDCODED
  oscar.StrafeRight(testSpeed);
  delay(9000);
  oscar.StrafeLeft(testSpeed);
  delay(9000);
  oscar.DriveBackward(testSpeed);
  delay(11000);
  oscar.StrafeRight(testSpeed);
  delay(9000);
  oscar.StrafeLeft(testSpeed);
  delay(1000);
  oscar.DriveBackward(testSpeed);
  delay(8000);
  oscar.StrafeLeft(testSpeed);
  delay(9000);
  oscar.DriveForward(255);
  delay(22000);
  oscar.StrafeRight(testSpeed);
  delay(9000);

/* TESTS:
 *  detect floor with one sensor
 *  turn around
 *  see pressure plate
 *  hit pressure plate from different angles
 *  
 * DISCOVERIES:
 * follow wall works (mostly could only test with long + short)
 * follow floor works but is not safe since strafe is not straight
 * pressure plate clicks when driving into
 * see pressure plate with sensors
 * 
 * 
 * FIXMES: 
 * we need left floor sensor
 * we need another back close sensor
 * we need left side sensor
 * some portions of motor is flipped
 * robot flipped - buttons in worse position 
 */

 
  // perform every some ms -- shouldnt use delay but system timing
  while(true) {
    // specific sensor updates performed within phase
    if(phase == 1) {
      toDestinationA();
    } else if(phase == 2) {
      oneSensorRamp();
      
      //centerOnAndDownRamp();
    }
    
    delay(5); 
  }

//  if(phase == 1) {
//    toDestinationA();
//  } else if(phase == 2) {
//    centerOnRamp();
//  } else if(phase == 3) {
//    downRamp();
//  } else if(phase == 4) {
//    toDestinationBWall();
//  } else if(phase == 5) {
//    // driving over destination B in the process
//    toFlagWall();
//  } else if(phase == 6) {
//    centerOnFlag();
//  } else if(phase == 7) {
//    turnAround();
//  } else if(phase == 8) {
//    centerOnChest();
//  } else if(phase == 9) {
//    atopChest();
//  } else if(phase == 10) {
//    pickUpChest();
//  } else if(phase == 11) {
//    centerOnRamp2();
//  } else if(phase == 12) {
//    upRamp();
//  } else if(phase == 13) {
//    toDestinationA2();
//  } else {
//    // localization backup routine?
//    // just do not be here
//  }
}

void toDestinationA() {

  // Reid's
//  if(isButtonPressed()) {
//    phase++;
//    // drive away from button wall
//    if(treasureMap[0] == 0) {
//      oscar.StrafeRight(testSpeed);
//    } else {
//      oscar.StrafeLeft(testSpeed);
//    }
//    return;
//  }

  
  // read back LIDAR
  I2CSelect(2, 2);
  double sensorBackRight = ReadShort() + 10.0;
  I2CSelect(2, -1);

//  filter1[2] = filter1[1];
//  filter1[1] = filter1[0];
//  filter1[0] = sensorBackRight;
//
//  double filtered1 = (filter1[2] + filter1[1] + filter1[0]) / 3.0;
  
  I2CSelect(3, 0);
  double sensorBackLeft = ReadLong();
  I2CSelect(3, -1);

//  filter2[2] = filter2[1];
//  filter2[1] = filter2[0];
//  filter2[0] = sensorBackLeft;
//
//  double filtered2 = (filter2[2] + filter2[1] + filter2[0]) / 3.0;

  // ID1: back close L, ID2: back close R, threshold: 8 mm
  int parallel = isParallel(sensorBackLeft, sensorBackRight, 10);
  
  // ID: back close L, desired: 40 mm, threshold: 8 mm
  int spacing = hasSpacing(sensorBackRight, 120, 10);

  if(parallel == tooCounter) {
    oscar.TurnLeft(testSpeed);
    return;
  }
  if(parallel == tooClock) {
    oscar.TurnRight(testSpeed);
    return;
  }

  // else is parallel
  if(spacing  == tooClose) {
    oscar.DriveBackward(testSpeed);
    return;
  } 

  // FIXME: always too far
  if(spacing == tooFar) {
    oscar.DriveForward(testSpeed);
    return;
  } 
 
  // else parallel and well-spaced to back wall
  if(treasureMap[0] == 0) {
      oscar.StrafeLeft(testSpeed);
  } else {
    oscar.StrafeRight(testSpeed);
  }
}

void centerOnAndDownRamp() {
  // read floor sensors
  I2CSelect(2, 2);
  double sensorFloorLeft = ReadShort();
  I2CSelect(2, -1);
  
  I2CSelect(3, 1);
  double sensorFloorRight = ReadShort();
  I2CSelect(3, -1);

  int leftSafe = hasSpacing(sensorFloorLeft, 150, 10);

  if(sensorFloorLeft > 30) {
    oscar.StrafeRight(testSpeed);
    return;
  }
  if(sensorFloorRight > 30) {
    oscar.StrafeLeft(testSpeed);
    return;
  }

  oscar.DriveForward(testSpeed);
}

//void centerOnRamp() {
//
//  // ID: back close, desired: 40 mm, threshold: 8 mm
//  int spacing = hasSpacing(4, 40, 8);
//  
//  // ID1: back close L, ID2: back close R, threshold: 8 mm
//  int parallel = isParallel(4, 5, 8);
//
//  if(spacing  == tooClose) {
//    moveForward();
//    return;
//  } 
//  if(spacing == tooFar) {
//    moveBack();
//    return;
//  } 
//  // else well-spaced from back wall
//  if(parallel == tooCounter) {
//    rotateClock();
//    return;
//  }
//  if(parallel == tooClock) {
//    rotateCounter();
//    return;
//  }
//  // else parallel to back wall 
//
//  // ID1: left far F, ID2: right far F, threshold: 50 mm
//  spacing = isCentered(8, 10, 50);
//
//  if(spacing == tooLeft) {
//    moveRight();
//    return;
//  }
//  if(spacing == tooRight) {
//    moveLeft();
//    return;
//  }
//  // else centered and ready to approach ramp
//  phase++;
//}
//
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

// INSERT: other phase functions

int hasSpacing(double sensor, float desired, float threshold) {

  float diff = sensor - desired;

  if(diff > threshold) {
    return tooFar;
  }
  if(diff < -threshold) {
    return tooClose;
  }
  return good;
}

int isParallel(double sensor1, double sensor2, float threshold) {

  float diff = sensor1 - sensor2;

  if(diff > threshold) {
    return tooCounter;
  }
  if(diff < -threshold) {
    return tooClock;
  }
  return good;
}

int isCentered(int ID1, int ID2, float threshold) {
  // same as isParallel
  // renamed to improve high level readability
  float diff = sensors[ID1] - sensors[ID2];

  if(diff > threshold) {
    return tooLeft;
  }
  if(diff < -threshold) {
    return tooRight;
  }
  return good;
}

void readIRSensor() {
  unsigned long irCode;
  if (myReceiver.getResults()) {
    myDecoder.decode();
    irCode = myDecoder.value;
    Serial.println(irCode, HEX);
    // receiving route signal
    if (!calibrationSignal)
    {
      switch(irCode) {
        case(0X1AF66ED4):
          sevSegDisplayNumber = 1;
          treasureMap[0] = 0;
          treasureMap[1] = 0;
          treasureMap[2] = 0;
          break;
        case(0X17F66A1D):
          sevSegDisplayNumber = 2;
          treasureMap[0] = 1;
          treasureMap[1] = 0;
          treasureMap[2] = 0;
          break;
        case(0XA4E2155E):
          sevSegDisplayNumber = 3;
          treasureMap[0] = 0;
          treasureMap[1] = 1;
          treasureMap[2] = 0;
          break;
        case(0XA3E213CD):
          sevSegDisplayNumber = 4;
          treasureMap[0] = 1;
          treasureMap[1] = 1;
          treasureMap[2] = 0;
          break;
        case(0XA8726262):
          sevSegDisplayNumber = 5;
          treasureMap[0] = 0;
          treasureMap[1] = 0;
          treasureMap[2] = 1;
          break;
        case(0XA97263F7):
          sevSegDisplayNumber = 6;
          treasureMap[0] = 1;
          treasureMap[1] = 0;
          treasureMap[2] = 1;
          break;
        case(0XB490A256):
          sevSegDisplayNumber = 7;
          treasureMap[0] = 0;
          treasureMap[1] = 1;
          treasureMap[2] = 1;
          break;
        case(0XB390A0C5):
          sevSegDisplayNumber = 8;
          treasureMap[0] = 1;
          treasureMap[1] = 1;
          treasureMap[2] = 1;
          break;
      }
      // write route to 7-segment display
      matrix.writeDigitRaw(3, 0B000000000);
      matrix.blinkRate(0);
      matrix.drawColon(false);
      matrix.writeDigitNum(4, sevSegDisplayNumber, true);
      matrix.writeDisplay();
    }
    // receiving calibration signal
    else {
      if(irCode == 0X1AF66ED4) {
        sevSegDisplayNumber = 0;  
        // write GO to 7-segment display
        matrix.blinkRate(0);
        matrix.drawColon(false);
        matrix.writeDigitRaw(3, 0B000111101);
        matrix.writeDigitNum(4, 0, false);
        matrix.writeDisplay();
      }
      else {
        sevSegDisplayNumber = 9; //arbitrary value  
      }
    }
    Serial.println(sevSegDisplayNumber);
    myReceiver.enableIRIn(); // Receive the next value 
  }
  delay(100);
  
}

// called when kill button is pressed
void killFunction() {
  //stop moving
  //stop motors
  Serial.println("Kill button pressed");
  matrix.writeDigitNum(4, 1, false);
  matrix.writeDisplay();
  // do nothing forever 
  while(1);
}

void setupLidar(){

  Wire.begin(); // join i2c bus as master
  I2CSelect(1, -1);
  I2CSelect(2, -1);
  I2CSelect(3, -1);

  Serial.println("BOARD 1");
  I2CSelect(1, 0);
  shortRange.begin();
  Serial.println("0");
  I2CSelect(1, 1);
  shortRange.begin();
  Serial.println("1");
  I2CSelect(1, 2);
  longRange.begin();
  Serial.println("2");
  I2CSelect(1, 3);
  longRange.begin();
  Serial.println("3");
  I2CSelect(1, 4);
  shortRange.begin(); //longRange.begin();
  Serial.println("4");
  I2CSelect(1, 5);
  shortRange.begin(); //longRange.begin();
  Serial.println("5");
  I2CSelect(1, -1);

  Serial.println("BOARD 2");
  I2CSelect(2, 0);
  longRange.begin();
  Serial.println("0");
  I2CSelect(2, 1);
  shortRange.begin();
  Serial.println("1");
  I2CSelect(2, 2);
  shortRange.begin();
  Serial.println("2");
  I2CSelect(2, 3);
  shortRange.begin();
  Serial.println("3");
  I2CSelect(2, 4);
  longRange.begin();
  Serial.println("4");
  I2CSelect(2, 5);
  longRange.begin();
  Serial.println("5");  // FIXME: 
  I2CSelect(2, -1);

  Serial.println("BOARD 3");
  I2CSelect(3, 0);
  longRange.begin();
  Serial.println("0");
  I2CSelect(3, 1);
  longRange.begin();
  Serial.println("1");
  I2CSelect(3, 2);
  shortRange.begin();
  Serial.println("2");
  I2CSelect(3, 3);
  shortRange.begin();
  Serial.println("3");
  I2CSelect(3, 4);
  shortRange.begin();
  Serial.println("4");
  I2CSelect(3, -1);
}

void I2CSelect(int mux, int8_t i) {
  if (i > 5) {
    Serial.println("Returning from tacselect.");
    return;
  }
  if(mux == 1){
    Wire.beginTransmission(TCAADDR1);
  }
  else if(mux == 2){
    Wire.beginTransmission(TCAADDR2);
  }
  else{
    Wire.beginTransmission(TCAADDR3);
  }

  if(i == -1){
    Wire.write(0);
  }
  else{
    Wire.write(1 << i);
  }
  
  Wire.endTransmission(); 
  return; 
}

float ReadShort(){
  float lux = shortRange.readLux(VL6180X_ALS_GAIN_5);
  uint8_t range = shortRange.readRange();
  uint8_t status = shortRange.readRangeStatus();
  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);
  }

  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
  return range;
}

float ReadLong(){
  VL53L0X_RangingMeasurementData_t measure;
  longRange.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  return measure.RangeMilliMeter;
}

//void rileyMaps() {
//
//
//  // wait until start button is pressed
//  while (digitalRead(startButton) == HIGH) {
//    // receive calibration IR signal
//    readIRSensor();
//  }
//  
//  // Riley
//  calibrationSignal = false;
//  readIRSensor();
//
//  matrix.blinkRate(0);
//  matrix.drawColon(false);
//  matrix.writeDigitNum(4, 0, false);
//  matrix.writeDisplay();
//}

void lidarReadings() {

  Serial.println("BOARD 1");
  Serial.println("\nSensor 0");
  I2CSelect(1,0);
  ReadShort();

  Serial.println("\nSensor 1");
  I2CSelect(1,1);
  ReadShort();

  Serial.println("\nSensor 2");
  I2CSelect(1,2);
  ReadLong();

  Serial.println("\nSensor 3");
  I2CSelect(1,3);
  ReadLong();

  Serial.println("\nSensor 4");
  I2CSelect(1,4);
  ReadLong();
  Serial.println("\nSensor 5");
  I2CSelect(1,5);
  ReadLong();
  I2CSelect(1,-1);
  

  Serial.println("\n\nBOARD 2");
  Serial.println("Sensor 0");
  I2CSelect(2,0);
  ReadLong();

  Serial.println("Sensor 1");
  I2CSelect(2,1);
  ReadShort();

  Serial.println("\nSensor 2");
  I2CSelect(2,2);
  ReadShort();

  Serial.println("Sensor 3");
  I2CSelect(2,3);
  ReadShort();

  Serial.println("Sensor 4");
  I2CSelect(2,4);
  ReadLong();

  Serial.println("Sensor 5");
  I2CSelect(2,5);
  ReadLong();
  I2CSelect(2,-1);

  Serial.println("\n\nBOARD 3");
  Serial.println("Sensor 0");
  I2CSelect(3,0);
  ReadLong();
  
  Serial.println("Sensor 1");
  I2CSelect(3,1);
  ReadLong();
  I2CSelect(3,-1);

  Serial.println("Sensor 2");
  I2CSelect(3,2);
  ReadLong();
  I2CSelect(3,-1);

  Serial.println("Sensor 3");
  I2CSelect(3,3);
  ReadLong();
  I2CSelect(3,-1);

  Serial.println("Sensor 4");
  I2CSelect(3,4);
  ReadLong();
  I2CSelect(3,-1);

  Serial.println("\n\n\n");
  delay(5000);

  
}

void oneSensorRamp() {

 I2CSelect(1,1);
 double sensorFloorRight = ReadShort();
 I2CSelect(1, -1);
  Serial.println(sensorFloorRight);
 if(sensorFloorRight > 90) {
   oscar.StrafeLeft(testSpeed);
   
   return;
 }

  oscar.DriveBackward(testSpeed);
}

void seePressure() {
  I2CSelect(2, 3);
  double sensorRight = ReadShort();
  I2CSelect(2, -1);
  Serial.println(sensorRight);
}

void followRightWallFar() {

  I2CSelect(2, 0);
  double sensorRightFront = ReadLong();
  I2CSelect(2, -1);
  
  I2CSelect(2, 4);
  double sensorRightBack = ReadLong();
  I2CSelect(2, -1);

  // ID1: back close L, ID2: back close R, threshold: 8 mm
  int parallel = isParallel(sensorRightFront, sensorRightBack, 10);
  
  // ID: back close L, desired: 40 mm, threshold: 8 mm
  int spacing = hasSpacing(sensorRightFront, 300, 10);

  if(parallel == tooCounter) {
    Serial.println("turn left");
    oscar.TurnLeft(testSpeed);
    return;
  }
  if(parallel == tooClock) {
    Serial.println("turn right");
    oscar.TurnRight(testSpeed);
    return;
  }

  // else is parallel
  if(spacing  == tooClose) {
    Serial.println("left");
    oscar.StrafeLeft(testSpeed);
    return;
  } 

  // FIXME: always too far
  if(spacing == tooFar) {
    Serial.println("right");
    oscar.StrafeRight(testSpeed);
    return;
  } 
 
  // else parallel and well-spaced to back wall
  if(treasureMap[0] == 0) {
      oscar.DriveForward(testSpeed);
    Serial.println("34985720983475092347");
  } else {
    oscar.StrafeRight(testSpeed);
    Serial.println("HEREHELRHLKSDHFLH:LFEHL");
  }
}
