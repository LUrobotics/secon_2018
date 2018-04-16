/**********************************************************************************************
 * Motor Interface for the Lipscomb IEEE 2018 Robotics Project
 * This code runs on an Adafruit Feather M0 Express. It controls two of the wheel motors via a 
 * MC33926 Motor Driver Shield.
 * 
 * Helpful Links:
 *    Feather  Guide: https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/overview
 *    Motor Controller: https://www.pololu.com/product/2503
 **********************************************************************************************/
#include "motor_interface.h"
#include "Adafruit_VL6180X.h" // https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/wiring-and-test
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define TCAADDR1 0x71
#define TCAADDR2 0x72
#define TCAADDR3 0x73

Adafruit_VL6180X shortRange = Adafruit_VL6180X();
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);
float data = 1000.0;

void setup() {
  Serial.begin(9600);
  MotorsInit();
  Wire.begin(); // join i2c bus as master
  I2CSelect(2, 1);
  shortRange.begin();
  Serial.println("1");
  
  delay(1000);
  
  DriveForward(30.0);
}

void loop() {
  Serial.println("\nSensor 1");
  I2CSelect(2,1);
  data = ReadShort();
  if(data < 100){
    Stop();
    Serial.println("Too close!");
  }
//  delay(50);
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
    return range;
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
  return 9999;
}
