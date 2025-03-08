#pragma once
#include <BatterySensor.h>
#include <IMU.h>
#include <ServoControllerPca9685.h>

class RobotIO {
 public:
  void begin();

  ServoControllerPca9685 servo;
  IMU imu;
  BatterySensor battery;
 private:
};