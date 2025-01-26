#pragma once

class BatterySensor {
 public:
  void begin();
  float readVoltage();
  int readPercentage();
  float readCurrent();

 private:
};