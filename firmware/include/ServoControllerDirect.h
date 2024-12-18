#pragma once
#include <ESP32Servo.h>

class ServoControllerDirect {
 public:
  void begin();
  void writeMicroseconds();

 private:
  Servo servos[12];
};