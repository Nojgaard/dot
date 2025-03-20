#pragma once
#include <Specs.h>
#include <RobotIO.h>
#include <helper_3dmath.h>
#include <AsyncUDP.h>

struct ServoMicrosecondsPacket {
  // Pulse width signal for each servo in microseconds.
  int microseconds[Specs::NUM_SERVOS];
};

struct ServoTargetAnglePacket {
  // Target angles for each servo in degrees.
  float angles[Specs::NUM_SERVOS];
};

struct ServoCalibrationPacket {
  float degreesPerSecond;
  float smoothingScalar;
  int minMicroseconds[Specs::NUM_SERVOS];
  int maxMicroSeconds[Specs::NUM_SERVOS];
  int maxAngles[Specs::NUM_SERVOS];
};

struct TelemetryPacket {
  float batteryVoltage;
  float batteryCurrent;
  VectorFloat orientation;
  VectorFloat acceleration;
  uint8_t statusIMU;
  uint8_t statusServo;
};

uint8_t readPacket(AsyncUDPPacket& packet, RobotIO& robotIO);