#pragma once
#include <ServoControllerDirect.h>
#include <ServoControllerPca9685.h>

class ServoController {
 public:
  void begin();
  void setCalibration(float degreesPerSecond, float smoothingScalar,
                      int* minMicroseconds, int* maxMicroseconds,
                      int* maxAngles);
  void setTargetAngle(float* angles);
  void setMicroseconds(int* microseconds);
  bool hasCalibration() const;
  void actuate(float dt);
  void detach();
  uint8_t deviceStatus() const;
  void printCalibration() const;
  float getCurrentAngle(int servoId) const;

 private:
  struct ServoData {
    float targetAngle = -1;
    float currentAngle = -1;
    int minMicroseconds = 500;
    int maxMicroseconds = 2500;
    int maxAngle = 180;
    float slope = (2500.0 - 500) / 180;
  };

  ServoData _servos[Specs::NUM_SERVOS];
  float _degreesPerSecond = 200;
  float _smoothingScalar = 100;
  bool _hasCalibration = false;

  ServoControllerPca9685 _device;
};