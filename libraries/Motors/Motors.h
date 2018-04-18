/**********************************************************************************************
 * Motor Driver for the Lipscomb IEEE 2018 Robotics Project
 * Created by Cailey Cline on April 7, 2018
 * This code runs on an Adafruit Feather M0 Express. It controls four wheel motors via two
 * MC33926 Motor Driver Shields.
 *
 * Helpful Links:
 *    Feather  Guide: https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/overview
 *    Motor Controller General Purpose Guide: https://www.pololu.com/docs/0J55/4
 **********************************************************************************************/

#ifndef Motors_h
#define Motors_h
#include "Arduino.h"

class Motors{
  public:
    Motors();
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
    void SquareDance(float speed);
};

#endif
