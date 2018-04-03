/**********************************************************************************************
 * LIDAR Hub code for the Lipscomb IEEE 2018 Robotics Project
 * This code runs on an Adafruit Trinket M0. It monitors several Adafruit VL53L0X and VL6180X
 * LIDAR sensors over I2C. All VL53L0X and VL6180X have I2C address 0x29, so two (three?) Adafruit 
 * TCA9548A 1-to-8 I2C Multiplexer Breakout boards are used to communicate with all of them.
 * 
 * Helpful Links:
 *    Trinket  Guide: https://learn.adafruit.com/adafruit-trinket-m0-circuitpython-arduino/overview
 *    VL53L0X  Guide: https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/overview
 *    VL6180X  Guide: https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/overview
 *    TCA9548A Guide: https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout?view=all
 * 
 **********************************************************************************************/
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL6180X.h"
#include <Wire.h>

Adafruit_VL6180X short1 = Adafruit_VL6180X();
Adafruit_VL53L0X long1 = Adafruit_VL53L0X();

void setup() {
  // initialize comm
  Serial.begin(9600);
  Serial.println("Hello!");
  // initialize sensors
  

}

void loop() {
  // repeatedly update sensors via I2C

}


void ISR_DataRequest(){
  // send requested data to Feather via serial connection
}

