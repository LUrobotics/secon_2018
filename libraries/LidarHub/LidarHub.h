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
#ifndef LidarHub_h
#define LidarHub_h
#include "Arduino.h"

class LidarHub{
  public:
    LidarHub();
    bool I2CSelect(int mux, int8_t sensorNum);
    float ReadShort();
    float ReadLong();

  private:
    Adafruit_VL6180X shortRange;
    Adafruit_VL53L0X longRange;
    int TCAADDR1 = 0x71;
    int TCAADDR2 = 0x72;
    int TCAADDR3 = 0x73;
};

#endif
