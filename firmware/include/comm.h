#pragma once

#include <AsyncUDP.h>

struct ServoPacket {
  int microseconds[12];
};

struct TelemetryPacket {
  float batteryVoltage;
  float batteryCurrent;
};

class Comm {
 public:
  bool begin();
  const ServoPacket& consumeServoPacket();
  void sendSensorState(const TelemetryPacket& packet);
  bool isServoPacketAvailable();
  bool isConnected();
  bool isTimedOut();

 private:
  bool connectToWifi();
  bool connectToController();
  void setupPacketReceiver();

  AsyncUDP udp;
  bool hasServoPacket;
  unsigned long timeoutMs = 1000;
  unsigned long lastReceivedPacketMs;
  unsigned long lastSentSensorPacketMs;
  ServoPacket lastServoPacket;
};