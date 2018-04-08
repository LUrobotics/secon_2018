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
#include "Adafruit_VL53L0X.h" // https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code
#include "Adafruit_VL6180X.h" // https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/wiring-and-test
#include <Wire.h>

#define TCAADDR0 0x70
#define TCAADDR1 0x71
#define TCAADDR2 0x72

Adafruit_VL6180X shortRange = Adafruit_VL6180X();
Adafruit_VL53L0X longRange = Adafruit_VL53L0X();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  while(!Serial);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Hello!");
  
  Wire.begin(); // join i2c bus as master
  I2CSelect(0, -1);
  I2CSelect(1, -1);
  I2CSelect(2, -1);
  
  I2CSelect(0, 0);
  shortRange.begin();
  I2CSelect(0, 1);
  shortRange.begin();
  I2CSelect(0, -1);
  I2CSelect(1, 0);
  longRange.begin();
  I2CSelect(1, 1);
  longRange.begin();
  I2CSelect(1, -1);
  I2CSelect(2, 0);
  longRange.begin();
  I2CSelect(2, 2);
  shortRange.begin();
  I2CSelect(2, 3);
  shortRange.begin();
  I2CSelect(2, -1);
}

void loop() {
  
  Serial.println("\nSensor 0");
  I2CSelect(0,0);
  ReadShort();
  delay(100);

  Serial.println("\nSensor 1");
  I2CSelect(0,1);
  ReadShort();
  delay(100);

  Serial.println("\nSensor 2");
  I2CSelect(0,-1);
  I2CSelect(1,0);
  ReadLong();
  delay(100);

  Serial.println("\nSensor 3");
  I2CSelect(1,1);
  ReadLong();
  delay(100);

  Serial.println("\nSensor 4");
  I2CSelect(1,-1);
  I2CSelect(2,0);
  ReadLong();
  delay(100);

  Serial.println("\nSensor 5");
  I2CSelect(2,3);
  ReadShort();
  delay(100);

  Serial.println("\nSensor 6");
  I2CSelect(2,2);
  ReadShort();
  delay(100);
  I2CSelect(2,-1);

  delay(2000); 

}
 
void I2CSelect(int mux, int8_t i) {
  if (i > 7) {
    Serial.println("Returning from tacselect.");
    return;
  }
  if(mux == 0){
    Wire.beginTransmission(TCAADDR0);
  }
  else if(mux == 1){
    Wire.beginTransmission(TCAADDR1);
  }
  else{
    Wire.beginTransmission(TCAADDR2);
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
  return range;
}

float ReadLong(){
  VL53L0X_RangingMeasurementData_t measure;
  longRange.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  return measure.RangeMilliMeter;
}

