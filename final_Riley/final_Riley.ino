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
//#include <Motors.h>
//#include <LidarHub.h>

#define tooClose    1
#define tooFar      2
#define good        0
#define tooCounter  1
#define tooClock    2
#define tooLeft     1 
#define tooRight    2

#define testSpeed   30

/* START & KILL BUTTON STUFF */
const int killButton = SCK;
const int startButton = A5;
bool startProgram = false;

/* CAPTAIN'S WHEEL TRINKET ENABLE PIN */
int capWheelTrinket = 6;
int armTrinket = MOSI;

/* IR & 7 SEG STUFF */
const int irRecvPin = A4;
IRdecode myDecoder;
IRrecvLoop myReceiver(irRecvPin);
int sevSegDisplayNumber = 0;
bool calibrationSignal = true;
Adafruit_7segment matrix = Adafruit_7segment();

//Motors oscar = Motors();
//LidarHub lidar = LidarHub();

/* NAV STUFF */
int phase = 1;
double sensors[14];
int treasureMap[] = {-1, -1, -1};

void setup() {
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("Waiting to start");
  // initialize start and kill buttons
  pinMode(killButton, INPUT_PULLUP); // pin = HIGH when switch open and LOW when switch is pressed
  pinMode(startButton, INPUT_PULLUP);
  // kill button interrupt
  attachInterrupt(digitalPinToInterrupt(killButton), killFunction, FALLING);
  // start button interrupt: sets flag to start program
  attachInterrupt(digitalPinToInterrupt(startButton), startFunction, FALLING);
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
  // make captain's wheel pin an output
  pinMode(capWheelTrinket, OUTPUT);
  digitalWrite(capWheelTrinket, HIGH);
  // make arm pin an output 
  pinMode(armTrinket, OUTPUT);
  digitalWrite(armTrinket, HIGH);
}

void loop() {
  // wait until start button is pressed
  while (!startProgram) {
    // receive calibration IR signal
    readIRSensor();
  }
  
  /* ENTER PROGRAM */
  //digitalWrite(capWheelTrinket, LOW); // DO THIS TO MAKE WHEEL TURN
  //digitalWrite(armTrinket, LOW); // DO THIS TO MAKE ARM MOVE
  
  // Riley
  calibrationSignal = false;
  readIRSensor();

  // perform every 50 ms
//  if(!is50ms()) {
//    return;
//  }

//  treasureMap[0] = 0;
//  treasureMap[1] = 0;
//  treasureMap[2] = 0;
//
//  phase = 1;
//  
//  while(true) {
//    if(phase == 1) {
//      toDestinationA();
//    } else if(phase == 2) {
//      centerOnRamp();
//    } else if(phase == 3) {
//      downRampBlind();
//    } else if(phase == 4) {
//      seeChest();
//    } else if(phase == 5) {
//      toPressureWall();
//    } else if(phase == 6) {
//      seePressurePlate();
//    }
//
//    // FIXME: lazy 
//    delay(5);
//  }

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

//void toDestinationA() {
//
//
////  // Reid's
////  if(isButtonPressed()) {
////    phase++;
////    return;
////  }
//  
//  // read back short sensors
//  lidar.I2CSelect(1,0);
//  double backLeftShort = lidar.ReadShort();
//  lidar.I2CSelect(1,-1);
//  
//  lidar.I2CSelect(2,1);
//  double backRightShort = lidar.ReadShort();
//  lidar.I2CSelect(2,-1);
//
//  // threshold: 8 mm
//  int parallel = isParallel(backLeftShort, backRightShort, 8);
//  
//  // desired: 120 mm, threshold: 8 mm
//  int spacing = hasSpacing(backLeftShort, 120, 8);
//  
//   if(parallel == tooLeft) {
//    oscar.TurnRight(testSpeed);
//    return;
//  }
//  if(parallel == tooRight) {
//    oscar.TurnLeft(testSpeed);
//    return;
//  }
//  // else parallel to wall
//  if(spacing  == tooClose) {
//    oscar.DriveForward(testSpeed);
//    return;
//  } 
//  if(spacing == tooFar) {
//    oscar.DriveBackward(testSpeed);
//    return;
//  } 
//  
//  // else parallel and well-spaced to back wall but button not hit
//  if(treasureMap[0] == 0) {
//    oscar.StrafeLeft(testSpeed);
//  } else {
//    oscar.StrafeRight(testSpeed);
//  }
//
//}
//
//void centerOnRamp() {
//  /*STAY PARALLEL AND WELL SPACED TO BACK WALL*/
//  // read back short sensors
//  lidar.I2CSelect(1,0);
//  double backLeftShort = lidar.ReadShort();
//  lidar.I2CSelect(1,-1);
//  
//  lidar.I2CSelect(2,1);
//  double backRightShort = lidar.ReadShort();
//  lidar.I2CSelect(2,-1);
//
//  // threshold: 8 mm
//  int parallel = isParallel(backLeftShort, backRightShort, 8);
//  
//  // desired: 120 mm, threshold: 8 mm
//  int spacing = hasSpacing(backLeftShort, 120, 8);
//  
//   if(parallel == tooLeft) {
//    oscar.TurnRight(testSpeed);
//    return;
//  }
//  if(parallel == tooRight) {
//    oscar.TurnLeft(testSpeed);
//    return;
//  }
//  // else parallel to wall
//  if(spacing  == tooClose) {
//    oscar.DriveForward(testSpeed);
//    return;
//  } 
//  if(spacing == tooFar) {
//    oscar.DriveBackward(testSpeed);
//    return;
//  } 
//
//  /*CENTER BETWEEN SIDE YELLOW WALLS*/
//  // read left side and right side far sensors
//  lidar.I2CSelect(2,0);
//  double leftFar = lidar.ReadLong();
//  lidar.I2CSelect(2,-1);
//  
//  lidar.I2CSelect(1,4);
//  double rightFar = lidar.ReadLong();
//  lidar.I2CSelect(1,-1);
//
//  // 8 mm away from center
//  int centered = isCentered(leftFar, rightFar, 8);
//
//  if(centered  == tooLeft) {
//    oscar.StrafeRight(testSpeed);
//    return;
//  } 
//  if(centered == tooRight) {
//    oscar.StrafeLeft(testSpeed);
//    return;
//  } 
//
//  // else centered and ready to approach ramp
//  phase++;
//}
//
//void downRampBlind() {
//  /* FORWARD UNTIL WE SEE CHEST -- however we (likely) see floor due to ramp angle*/
//  // FIXME: lazy
//  oscar.DriveForward(testSpeed);
//  delay(5000);
//  phase++;
//}
//
//void seeChest() {
//  oscar.DriveForward(testSpeed);
//}
//
//void toPressureWall() {
//  
//}
//
//void seePressurePlate() {
//  
//}

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
//int hasSpacing(double reading, float desired, float threshold) {
//
//  float diff = reading - desired;
//
//  if(diff > threshold) {
//    return tooFar;
//  }
//  if(diff < -threshold) {
//    return tooClose;
//  }
//  return good;
//}
//
//int isParallel(double left, double right, float threshold) {
//
//  float diff = left - right;
//
//  if(diff > threshold) {
//    return tooLeft;
//  }
//  if(diff < -threshold) {
//    return tooRight;
//  }
//  return good;
//}
//
//int isCentered(double left, double right, float threshold) {
//  // same as isParallel just meant to match sensors on opposite sides of robot
//  float diff = left - right;
//
//  if(diff > threshold) {
//    return tooLeft;
//  }
//  if(diff < -threshold) {
//    return tooRight;
//  }
//  return good;
//}

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

void startFunction() {
  startProgram = true; 
}

// called when kill button is pressed
void killFunction() {
  //stop moving
  //stop motors
  Serial.println("Kill button pressed"); 
  // do nothing forever 
  while(1);
}
