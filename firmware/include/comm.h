#pragma once

#include <AsyncUDP.h>
#include <helper_3dmath.h>
#include <RobotIO.h>
#include <Packets.h>

class Comm {
 public:
  bool begin(RobotIO& robotIO);
  void sendSensorState(const TelemetryPacket& packet);
  bool isConnected();
  bool isTimedOut();

 private:
  bool connectToWifi();
  bool connectToController(RobotIO& robotIO);
  void setupPacketReceiver(RobotIO& robotIO);

  AsyncUDP udp;
  bool hasServoPacket;
  unsigned long timeoutMs = 1000;
  unsigned long lastReceivedPacketMs;
  unsigned long lastSentSensorPacketMs;
};