/*
 * Start button, kill button, IR, and trinket communication code
 */
 
#include <IRLibDecodeBase.h> 
#include <IRLib_HashRaw.h>  //Must be last protocol
#include <IRLibCombo.h>     // After all protocols, include this
#include <IRLibRecvLoop.h> 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

/* START & KILL BUTTON STUFF */
const int killButton = SCK;
const int startButton = A5;
bool startProgram = false;

/* CAPTAIN'S WHEEL AND ARM TRINKETS' ENABLE PINS */
int capWheelTrinket = 6;
int armTrinket = MOSI;

/* IR & 7 SEG STUFF */
const int irRecvPin = A4;
IRdecode myDecoder;
IRrecvLoop myReceiver(irRecvPin);
int sevSegDisplayNumber = 0;
bool calibrationSignal = true;
Adafruit_7segment matrix = Adafruit_7segment();

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
  digitalWrite(capWheelTrinket, LOW); // set it low just to be sure
  // make arm pin an output 
  pinMode(armTrinket, OUTPUT);
  digitalWrite(armTrinket, LOW); // set it low just to be sure
}

void loop() {
  // wait until start button is pressed
  // it will only go through main loop once with this scheme
  while (!startProgram) {
    // receive calibration IR signal
    readIRSensor();
  }
  
  /* ENTER PROGRAM */

  /* TEST FOR COMMUNICATING WITH TRINKETS */
  //digitalWrite(capWheelTrinket, HIGH); // DO THIS TO MAKE WHEEL TURN
  //pinMode(capWheelTrinket, INPUT); // so trinket can tell feather when it's done
  //while(digitalRead(capWheelTrinket) == HIGH) {;} // do nothing while captain's wheel does its thing
  //digitalWrite(armTrinket, HIGH); // DO THIS TO MAKE ARM MOVE
  //pinMode(armTrinket, INPUT); // so trinket can tell feather when it's done
  //while(digitalRead(armTrinket) == HIGH) {;} // do nothing while arm does its thing
  
  // Riley
  calibrationSignal = false;
  readIRSensor();
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

// called when start button is pressed
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
