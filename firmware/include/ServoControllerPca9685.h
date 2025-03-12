#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <Specs.h>

class ServoControllerPca9685 {
 public:
  void begin();
  void writeMicroSeconds(int servoNum, int us);
  void writeMicroSeconds(const int us[Specs::NUM_SERVOS]);
  void detach();
  uint8_t deviceStatus() const;

 private:
  Adafruit_PWMServoDriver pwm;
  uint8_t _deviceStatus;
};