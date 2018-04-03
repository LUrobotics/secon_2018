/**********************************************************************************************
 * Motor Interface for the Lipscomb IEEE 2018 Robotics Project
 * This code runs on an Adafruit Trinket M0. It controlls two of the wheel motors via a MC33926 
 * Motor Driver Shield. The robot's main controller communicates with the Trinket via I2C. It provides
 * the Trinket with the motor ID and speed
 * 
 * Helpful Links:
 *    Trinket  Guide:   https://learn.adafruit.com/adafruit-trinket-m0-circuitpython-arduino/overview
 *    Motor Controller: https://www.pololu.com/product/2503
 **********************************************************************************************/
#include <Wire.h>
//#include "DualMC33926MotorShield.h"

void setup() {
//  Wire.begin(4); // join i2c bus with address #4
//  Wire.onReceive(IncomingRequest); // register i2c recieve event
  pinMode(3, OUTPUT);
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  analogWrite(3, 50); // 50% duty cycle
}

void loop() {
  

}

void IncomingRequest(int howMany){
  // howMany = the number of bytes being received
  // Receive message & decode as JSON 
//  while(1 < Wire.available()) // loop through all but the last
//  {
//    char c = Wire.read(); // receive byte as a character
//    Serial.print(c);         // print the character
//  }
//  int x = Wire.read();    // receive byte as an integer
//  Serial.println(x);         // print the integer

}
