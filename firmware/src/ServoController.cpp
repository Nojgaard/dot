#include <ServoController.h>
#include <Arduino.h>

void ServoController::begin() { _device.begin(); }

void ServoController::setCalibration(float degreesPerSecond,
                                     int* minMicroseconds, int* maxMicroseconds,
                                     int* maxAngles) {
  _degreesPerSecond = degreesPerSecond;

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

void ServoController::actuate(float dt) {
  if (!hasCalibration()) {
    return;
  }
  
  float dangle = dt * _degreesPerSecond;
  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    ServoData& servo = _servos[i];
    if (servo.targetAngle < 0 || abs(servo.targetAngle - servo.currentAngle) < 0.01) {
      continue;
    }

    if (servo.currentAngle < 0) {
        servo.currentAngle = servo.targetAngle;
    } else {
        float error = servo.targetAngle - servo.currentAngle;
        float dangleConstrained = min(abs(error), dangle);
        servo.currentAngle += error >= 0 ? dangleConstrained : -dangleConstrained;
    }

    int microseconds = round(servo.currentAngle * servo.slope + servo.minMicroseconds);

    _device.writeMicroSeconds(i, microseconds);
  }
}

void ServoController::detach() {
    _device.detach();
}

uint8_t ServoController::deviceStatus() const { 
    return hasCalibration() ? _device.deviceStatus() : 2; 
}