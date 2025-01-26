#pragma once
#include <ESP32Servo.h>
#include <Specs.h>

class ServoControllerDirect {
 public:
  void begin();
  void writeMicroSeconds(int servoNum, int us);
  void writeMicroSeconds(const int us[Specs::NUM_SERVOS]);
  void detach();

 private:
  Servo servos[12];
};