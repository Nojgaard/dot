#pragma once
#include <BatterySensor.h>
#include <IMU.h>
#include <ServoController.h>

class RobotIO {
 public:
  void begin();
  void printStatus() const;

  ServoController servo;
  IMU imu;
  BatterySensor battery;
 private:
};