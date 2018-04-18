/**********************************************************************************************
 * Motor Driver for the Lipscomb IEEE 2018 Robotics Project
 * Created by Cailey Cline on April 7, 2018
 * This code runs on an Adafruit Feather M0 Express. It controls four wheel motors via two
 * MC33926 Motor Driver Shields.
 *
 * Programming Notes:
 *    x_DRIVE pins output a PWM signal which controls the speed of the motors. x_DIR pins
 *    output a binary signal which controls the direction of the motors. (Forward is LOW
 *    (default), Backward is HIGH)
 *
 * Helpful Links:
 *    Feather  Guide: https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/overview
 *    Motor Controller General Purpose Guide: https://www.pololu.com/docs/0J55/4
 **********************************************************************************************/

#include "Arduino.h"
#include "Motors.h"

#define MFL_DRIVE 12
#define MFR_DRIVE 10
#define MBL_DRIVE 13
#define MBR_DRIVE 11
#define MFL_DIR A1
#define MFR_DIR A3
#define MBL_DIR A0
#define MBR_DIR A2
#define FORWARD LOW
#define BACKWARD HIGH

Motors::Motors(){
  pinMode(MFL_DRIVE, OUTPUT);
  pinMode(MFR_DRIVE, OUTPUT);
  pinMode(MBL_DRIVE, OUTPUT);
  pinMode(MBR_DRIVE, OUTPUT);
  pinMode(MFL_DIR, OUTPUT);
  pinMode(MFR_DIR, OUTPUT);
  pinMode(MBL_DIR, OUTPUT);
  pinMode(MBR_DIR, OUTPUT);
}

void Motors::Stop(){
  analogWrite(MFL_DRIVE, 0);
  analogWrite(MFR_DRIVE, 0);
  analogWrite(MBL_DRIVE, 0);
  analogWrite(MBR_DRIVE, 0);
  return;
}

void Motors::DriveForward(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, FORWARD);
  digitalWrite(MFR_DIR, BACKWARD);
  digitalWrite(MBL_DIR, FORWARD);
  digitalWrite(MBR_DIR, BACKWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void Motors::DriveBackward(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, BACKWARD);
  digitalWrite(MFR_DIR, FORWARD);
  digitalWrite(MBL_DIR, BACKWARD);
  digitalWrite(MBR_DIR, FORWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void Motors::StrafeLeft(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, FORWARD);
  digitalWrite(MFR_DIR, FORWARD);
  digitalWrite(MBL_DIR, BACKWARD);
  digitalWrite(MBR_DIR, BACKWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, pulseWidth);
}


void Motors::StrafeRight(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, BACKWARD);
  digitalWrite(MFR_DIR, BACKWARD);
  digitalWrite(MBL_DIR, FORWARD);
  digitalWrite(MBR_DIR, FORWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void Motors::TurnLeft(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, BACKWARD);
  digitalWrite(MFR_DIR, BACKWARD);
  digitalWrite(MBL_DIR, BACKWARD);
  digitalWrite(MBR_DIR, BACKWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void Motors::TurnRight(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, FORWARD);
  digitalWrite(MFR_DIR, FORWARD);
  digitalWrite(MBL_DIR, FORWARD);
  digitalWrite(MBR_DIR, FORWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void Motors::DiagonalForwardRight(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, FORWARD);
  digitalWrite(MBR_DIR, FORWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, 0);
  analogWrite(MBL_DRIVE, 0);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void Motors::DiagonalForwardLeft(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFR_DIR, FORWARD);
  digitalWrite(MBL_DIR, FORWARD);
  analogWrite(MFL_DRIVE, 0);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, 0);
}

void Motors::DiagonalBackwardLeft(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, BACKWARD);
  digitalWrite(MBR_DIR, BACKWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, 0);
  analogWrite(MBL_DRIVE, 0);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void Motors::DiagonalBackwardRight(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFR_DIR, BACKWARD);
  digitalWrite(MBL_DIR, BACKWARD);
  analogWrite(MFL_DRIVE, 0);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, 0);
}


void Motors::SquareDance(float speed){
  DriveForward(speed);
  delay(1500);
  StrafeLeft(speed);
  delay(1500);
  DriveBackward(speed);
  delay(1500);
  StrafeRight(speed);
  delay(1500);
  Stop();
}
