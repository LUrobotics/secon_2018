#include "Adafruit_VL53L0X.h" // https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code
#include "Adafruit_VL6180X.h" // https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/wiring-and-test
#include <Wire.h>

Adafruit_VL6180X shortRange = Adafruit_VL6180X();
Adafruit_VL53L0X longRange = Adafruit_VL53L0X();
int TCAADDR1 = 0x71;
int TCAADDR2 = 0x72;
int TCAADDR3 = 0x73;
    


bool I2CSelect(int mux, int8_t sensorNum){
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

float ReadShort(){
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

float ReadLong(){
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

void muxInit(int mux){
  Serial.print("BOARD ");
  Serial.println(mux);
  I2CSelect(mux, 0);
//  Serial.println("Trying...");
  if(shortRange.begin()){
    Serial.println("0 - short");
  }
  else if(longRange.begin()){
    Serial.println("0 - long");
  }
  else{
    Serial.println("0 - SUCKS");
  }
  I2CSelect(mux, 1);
//  Serial.println("Trying...");
  if(shortRange.begin()){
    Serial.println("1 - short");
  }
  else if(longRange.begin()){
    Serial.println("1 - long");
  }
  else{
    Serial.println("1 - SUCKS");
  }
  I2CSelect(mux, 2);
//  Serial.println("Trying...");
  if(shortRange.begin()){
    Serial.println("2 - short");
  }
  else if(longRange.begin()){
    Serial.println("2 - long");
  }
  else{
    Serial.println("2 - SUCKS");
  }
  I2CSelect(mux, 3);
//  Serial.println("Trying...");
  if(shortRange.begin()){
    Serial.println("3 - short");
  }
  else if(longRange.begin()){
    Serial.println("3 - long");
  }
  else{
    Serial.println("3 - SUCKS");
  }
  I2CSelect(mux, 4);
//  Serial.println("Trying...");
  if(shortRange.begin()){
    Serial.println("4 - short");
  }
  else if(longRange.begin()){
    Serial.println("4 - long");
  }
  else{
    Serial.println("4 - SUCKS");
  }
  I2CSelect(mux, 5);
//  Serial.println("Trying...");
  if(shortRange.begin()){
    Serial.println("5 - short");
  }
  else if(longRange.begin()){
    Serial.println("5 - long");
  }
  else{
    Serial.println("5 - SUCKS");
  }
  I2CSelect(mux, -1);
}
