

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
#define BackRampTime  7800
#define IN1           0
#define IN2           MISO

/* START & KILL BUTTON STUFF */
const int killButton = SCK;
const int startButton = A5;
bool startProgram = false;

/* IR & 7 SEG STUFF */
const int irRecvPin = A4;
IRdecode myDecoder;
IRrecvLoop myReceiver(irRecvPin);
int sevSegDisplayNumber = 0;
bool calibrationSignal = true;
Adafruit_7segment matrix = Adafruit_7segment();

/* CAPTAIN'S WHEEL AND ARM TRINKETS' ENABLE PINS */
const int capWheelTrinket = 6;

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
  //while(!Serial);
  //Serial.println("Waiting to start");
  
  // initialize start and kill buttons
  pinMode(killButton, INPUT_PULLUP); // pin = HIGH when switch open and LOW when switch is pressed
  pinMode(startButton, INPUT_PULLUP);
  
  // arm pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
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
  
  // make captain's wheel interrupt pin an output
  pinMode(capWheelTrinket, OUTPUT);
  digitalWrite(capWheelTrinket, HIGH); // set it high just to be sure
  Wire.begin();

  muxInit(1);
  muxInit(2);
  muxInit(3);
}

void loop() {
  // wait until start button is pressed
  // it will only go through main loop once with this scheme
  while (!startProgram)
  {
    // receive calibration IR signal
    readIRSensor();
    Serial.println("waiting");
  }

  Serial.println("exited loop");
  
  // Serial.println("Start");
  
  /* ENTER PROGRAM */
  
  // Riley
  calibrationSignal = false;
  readIRSensor();
  

  // comment out for IR
  // treasureMap[0] = 1;
  // treasureMap[1] = 1;
  // treasureMap[2] = 1; 


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
      delay(1800);
      oscar.StrafeRight(testSpeed);
      delay(ACTime-175);
  } else {
      oscar.StrafeRight(testSpeed);
      delay(1800);
      oscar.StrafeLeft(testSpeed);
      delay(ACTime-175);
      }
  oscar.Stop();
  delay(100);
  oscar.DriveForward(testSpeed);
  delay(250);
  oscar.Stop();
  delay(500);

  // turn captain's wheel and lower arm slightly
  digitalWrite(capWheelTrinket, LOW);
  digitalWrite(IN1, HIGH);
  delay(1500);
  digitalWrite(IN1, LOW);
  delay(18500); // delay for captain's wheel to turn

  // drive to chest
  oscar.DriveBackward(testSpeed);
  delay(1500);
  oscar.Stop();
  delay(100);

  // pick up chest
  digitalWrite(IN1, HIGH);
  delay(200);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  oscar.DriveBackward(100);
  delay(1900);
  digitalWrite(IN2, LOW);
  oscar.DriveBackward(100);
  delay(BackRampTime-2100);
  oscar.Stop();

  // drive to and up ramp
  oscar.DriveForward(testSpeed);
  delay(Adjust);
  oscar.Stop();

  // add in to strafe left slightly before going up ramp
//  oscar.DriveBackward(100);
//  delay(BackRampTime/2);
//  oscar.Stop();
//  oscar.StrafeRight(testSpeed);
//  delay(30
//  oscar.Stop();
//  oscar.DriveBackward(100);
//  delay(BackRampTime/2);
//  oscar.Stop();
//  oscar.DriveBackward(100);
//  delay(BackRampTime);
//  oscar.Stop();
 
  // Desination C
  if(treasureMap[2] == 1){
  oscar.StrafeLeft(testSpeed);
  delay(ACTime+500);
  oscar.Stop();
  }
  else{
  oscar.StrafeRight(testSpeed);
  delay(ACTime+500);
  oscar.Stop();
  }
  while(1);
}

// call to read IR sensor
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
          treasureMap[0] = 0;
          treasureMap[1] = 0;
          treasureMap[2] = 1;
          break;
        case(0XA4E2155E):
          sevSegDisplayNumber = 3;
          treasureMap[0] = 0;
          treasureMap[1] = 1;
          treasureMap[2] = 0;
          break;
        case(0XA3E213CD):
          sevSegDisplayNumber = 4;
          treasureMap[0] = 0;
          treasureMap[1] = 1;
          treasureMap[2] = 1;
          break;
        case(0XA8726262):
          sevSegDisplayNumber = 5;
          treasureMap[0] = 1;
          treasureMap[1] = 0;
          treasureMap[2] = 0;
          break;
        case(0XA97263F7):
          sevSegDisplayNumber = 6;
          treasureMap[0] = 1;
          treasureMap[1] = 0;
          treasureMap[2] = 1;
          break;
        case(0XB490A256):
          sevSegDisplayNumber = 7;
          treasureMap[0] = 1;
          treasureMap[1] = 1;
          treasureMap[2] = 0;
          break;
        case(0XB390A0C5):
          sevSegDisplayNumber = 8;
          treasureMap[0] = 1;
          treasureMap[1] = 1;
          treasureMap[2] = 1;
          break;
        default:
          sevSegDisplayNumber = 9;
          // arbitrary values
          treasureMap[0] = 1;
          treasureMap[1] = 1;
          treasureMap[2] = 1;
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

// called when start button is pressed
void startFunction() {
  Serial.println("start button pressed");
  startProgram = true; 
}

// called when kill button is pressed
void killFunction() {
  //stop moving
  oscar.Stop();
  //stop motors
  Serial.println("Kill button pressed"); 
  // do nothing forever 
  while(1);
}


