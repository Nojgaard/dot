#include <ServoController.h>
#include <Arduino.h>

void ServoController::begin() { _device.begin(); }

void ServoController::setCalibration(float degreesPerSecond, float smoothingScalar,
                                     int* minMicroseconds, int* maxMicroseconds,
                                     int* maxAngles) {
  _degreesPerSecond = max(degreesPerSecond, 0.0f);
  _smoothingScalar = max(smoothingScalar, 0.0f);

  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    _servos[i].minMicroseconds = minMicroseconds[i];
    _servos[i].maxMicroseconds = maxMicroseconds[i];
    _servos[i].maxAngle = maxAngles[i];
    _servos[i].slope =
        (1.0f * maxMicroseconds[i] - minMicroseconds[i]) / maxAngles[i];
  }
  _hasCalibration = true;
}

bool ServoController::hasCalibration() const { return _hasCalibration; }

void ServoController::printCalibration() const {
    Serial.println("Servo calibration:");

    Serial.print("\tDegrees per second: ");
    Serial.println(_degreesPerSecond);

    Serial.print("\tMin microseconds: ");
    for (int i = 0; i < Specs::NUM_SERVOS; i++) {
        Serial.print(_servos[i].minMicroseconds);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("\tMax microseconds: ");
    for (int i = 0; i < Specs::NUM_SERVOS; i++) {
        Serial.print(_servos[i].maxMicroseconds);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("\tMax servo angles: ");
    for (int i = 0; i < Specs::NUM_SERVOS; i++) {
        Serial.print(_servos[i].maxAngle);
        Serial.print(" ");
    }
    Serial.println();
}

void ServoController::setMicroseconds(int* microseconds) {
  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    _servos[i].currentAngle = -1;
    _servos[i].targetAngle = -1;
  }
  _device.writeMicroSeconds(microseconds);
}

void ServoController::setTargetAngle(float* targetAngles) {
  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    _servos[i].targetAngle = targetAngles[i];
  }
}

float delta_angle_linear(float target, float current, float maxDeltaAngle) {
  float error = target - current;
  float dangle = min(abs(error), maxDeltaAngle);
  return error >= 0 ? dangle : -dangle;
}

float delta_angle_smoothed(float target, float current, float scalar) {
  return (target * (1 - scalar) + current * scalar) - current;
}

void ServoController::actuate(float dt) {
  if (!hasCalibration() || dt <= 0.0) {
    return;
  }
  float deltaMaxAngle = dt * _degreesPerSecond;
  float smoothFactor = 1.0 - exp(-(double)_smoothingScalar * (double)dt);
  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    ServoData& servo = _servos[i];
    if (servo.targetAngle < 0 || abs(servo.targetAngle - servo.currentAngle) < 0.01) {
      continue;
    }

    if (servo.currentAngle < 0) {
        servo.currentAngle = servo.targetAngle;
    } else {
      float dlinear = delta_angle_linear(servo.targetAngle, servo.currentAngle, deltaMaxAngle);
      float dsmooth = delta_angle_smoothed(servo.targetAngle, servo.currentAngle, smoothFactor);
      servo.currentAngle += abs(dlinear) < abs(dsmooth) ? dlinear : dsmooth;
    }

    int microseconds = round(servo.currentAngle * servo.slope + servo.minMicroseconds);

    _device.writeMicroSeconds(i, microseconds);
  }
}

float ServoController::getCurrentAngle(int servoId) const { return _servos[servoId].currentAngle; }

void ServoController::detach() {
    _device.detach();
}

uint8_t ServoController::deviceStatus() const { 
    return hasCalibration() ? _device.deviceStatus() : 2; 
}