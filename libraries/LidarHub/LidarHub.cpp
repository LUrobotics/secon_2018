/**********************************************************************************************
 * LIDAR Hub code for the Lipscomb IEEE 2018 Robotics Project
 * This code runs on an Adafruit Feather M0 Express. It monitors up to 18 Adafruit VL53L0X and VL6180X
 * LIDAR sensors over I2C. All VL53L0X and VL6180X have I2C address 0x29, so three Adafruit
 * TCA9548A 1-to-8 I2C Multiplexer Breakout boards are used to communicate with all of them.
 *
 * Helpful Links:
 *    Feather  Guide: https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/overview
 *    VL53L0X  Guide: https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/overview
 *    VL6180X  Guide: https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/overview
 *    TCA9548A Guide: https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout?view=all
 **********************************************************************************************/

#include "Arduino.h"
#include "LidarHub.h"
#include "Adafruit_VL53L0X.h" // https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code
#include "Adafruit_VL6180X.h" // https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/wiring-and-test
#include <Wire.h>

#define OFF -1

Adafruit_VL6180X shortRange = Adafruit_VL6180X();
Adafruit_VL53L0X longRange = Adafruit_VL53L0X();


LidarHub::LidarHub(){
  Wire.begin();
  I2CSelect(1, OFF);
  I2CSelect(2, OFF);
  I2CSelect(3, OFF);
}

bool LidarHub::I2I2CSelect(int mux, int8_t sensorNum){
  if (sensorNum > 5) return false;

  if(mux == 1){
    Wire.beginTransmission(TCAADDR1);
  }
  else if(mux == 2){
    Wire.beginTransmission(TCAADDR2);
  }
  else{
    Wire.beginTransmission(TCAADDR3);
  }

  if(sensorNum == -1){
    Wire.write(0); // turn I2C mux off
  }
  else{
    Wire.write(1 << sensorNum); // activate specified I2C sensor
  }

  Wire.endTransmission();
  return true;
}

float LidarHub::ReadShort(){
  uint8_t range = shortRange.readRange();

  if(Serial){ // If Serial monitor is being used, read and send error messages
    uint8_t status = shortRange.readRangeStatus();
    if (status == VL6180X_ERROR_NONE) {
      Serial.print("Range: "); Serial.println(range);
    }
    else if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
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
  }
  return range;
}

float LidarHub::ReadLong(){
  VL53L0X_RangingMeasurementData_t measure;
  longRange.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
  if(Serial){
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    }
    else {
      Serial.println(" out of range ");
    }
  }

  return measure.RangeMilliMeter;
}
