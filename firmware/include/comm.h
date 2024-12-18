#pragma once

#include <AsyncUDP.h>

struct ServoPacket {
  int microseconds[12];
};

class Comm {
 public:
  bool begin();
  const ServoPacket& consumeServoPacket();
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
  ServoPacket lastServoPacket;
};