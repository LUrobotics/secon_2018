/**********************************************************************************************
 * Motor Interface for the Lipscomb IEEE 2018 Robotics Project
 * This code runs on an Adafruit Feather M0 Express. It controls two of the wheel motors via a 
 * MC33926 Motor Driver Shield.
 * 
 * Helpful Links:
 *    Feather  Guide: https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/overview
 *    Motor Controller: https://www.pololu.com/product/2503
 **********************************************************************************************/

#define MFL_DRIVE 13
#define MFR_DRIVE 12
#define MBL_DRIVE 11
#define MBR_DRIVE 10
#define MFL_DIR A0
#define MFR_DIR A1
#define MBL_DIR A2
#define MBR_DIR A3
#define FORWARD LOW
#define BACKWARD HIGH

void MotorsInit();
void Stop();
void DriveForward(float speed);
void DriveBackward(float speed);
void StrafeRight(float speed);
void StrafeLeft(float speed);
void TurnLeft(float speed);
void TurnRight(float speed);
void DiagonalForwardRight(float speed);
void DiagonalForwardLeft(float speed);
void DiagonalBackwardRight(float speed);
void DiagonalBackwardLeft(float speed);


void MotorsInit(){
  pinMode(MFL_DRIVE, OUTPUT);
  pinMode(MFR_DRIVE, OUTPUT);
  pinMode(MBL_DRIVE, OUTPUT);
  pinMode(MBR_DRIVE, OUTPUT);
  pinMode(MFL_DIR, OUTPUT);
  pinMode(MFR_DIR, OUTPUT);
  pinMode(MBL_DIR, OUTPUT);
  pinMode(MBR_DIR, OUTPUT);
}

void Stop(){
  analogWrite(MFL_DRIVE, 0);
  analogWrite(MFR_DRIVE, 0);
  analogWrite(MBL_DRIVE, 0);
  analogWrite(MBR_DRIVE, 0);
  return;
}

void DriveForward(float speed){
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

void DriveBackward(float speed){
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

void StrafeRight(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, FORWARD);
  digitalWrite(MFR_DIR, BACKWARD);
  digitalWrite(MBL_DIR, BACKWARD);
  digitalWrite(MBR_DIR, FORWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, pulseWidth);
}


void StrafeLeft(float speed){
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

void TurnLeft(float speed){
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

void TurnRight(float speed){
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

void DiagonalForwardRight(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, FORWARD);
  digitalWrite(MBR_DIR, FORWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, 0);
  analogWrite(MBL_DRIVE, 0);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void DiagonalForwardLeft(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFR_DIR, FORWARD);
  digitalWrite(MBL_DIR, FORWARD);
  analogWrite(MFL_DRIVE, 0);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, 0);
}

void DiagonalBackwardLeft(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFL_DIR, BACKWARD);
  digitalWrite(MBR_DIR, BACKWARD);
  analogWrite(MFL_DRIVE, pulseWidth);
  analogWrite(MFR_DRIVE, 0);
  analogWrite(MBL_DRIVE, 0);
  analogWrite(MBR_DRIVE, pulseWidth);
}

void DiagonalBackwardRight(float speed){
  int pulseWidth = 255*(speed/100);
  digitalWrite(MFR_DIR, BACKWARD);
  digitalWrite(MBL_DIR, BACKWARD);
  analogWrite(MFL_DRIVE, 0);
  analogWrite(MFR_DRIVE, pulseWidth);
  analogWrite(MBL_DRIVE, pulseWidth);
  analogWrite(MBR_DRIVE, 0);
}


void SquareDance(float speed){
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

