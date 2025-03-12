#include <Packets.h>

enum PacketType {
  SERVO_MS = 1,
  SERVO_TARGET_ANGLE = 2,
  SERVO_CALIBRATION = 3,
  PING = 4
};

ServoMicrosecondsPacket servoMicroSecondsPacket;
ServoTargetAnglePacket servoTargetAnglePacket;
ServoCalibrationPacket servoCalibrationPacket;

template <typename T>
bool parsePacket(AsyncUDPPacket& packet, T& out) {
  if (packet.length() != sizeof(T) + 4) {
    Serial.println("Package size is incorrect");
    return false;
  }

  memcpy(&out, packet.data() + 4, sizeof(T));
  return true;
}

void readServoMicrosecondsPacket(ServoMicrosecondsPacket& packet,
                                 RobotIO& robotIO) {
  robotIO.servo.setMicroseconds(packet.microseconds);
}

void readServoTargetAnglePacket(ServoTargetAnglePacket& packet,
                                RobotIO& robotIO) {
  robotIO.servo.setTargetAngle(packet.angles);
}

void readServoCalibrationPacket(ServoCalibrationPacket& packet,
                                RobotIO& robotIO) {
  robotIO.servo.setCalibration(packet.degreesPerSecond, packet.minMicroseconds,
                               packet.maxMicroSeconds, packet.maxAngles);
  //robotIO.servo.printCalibration();
}

void readPingPacket(AsyncUDPPacket& packet) {
  Serial.println("Received ping packet");
  Serial.println(packet.remoteIP());
  Serial.println(packet.remotePort());
  Serial.println(packet.localIP());
  Serial.println(packet.localPort());
  packet.write(0);
}

uint8_t readPacket(AsyncUDPPacket& packet, RobotIO& robotIO) {
  uint8_t packetType = packet.data()[0];
  if (packetType == SERVO_MS) {
    if (!parsePacket(packet, servoMicroSecondsPacket)) return 2;
    readServoMicrosecondsPacket(servoMicroSecondsPacket, robotIO);
  } else if (packetType == SERVO_TARGET_ANGLE) {
    if (!parsePacket(packet, servoTargetAnglePacket)) return 2;
    readServoTargetAnglePacket(servoTargetAnglePacket, robotIO);
  } else if (packetType == SERVO_CALIBRATION) {
    if (!parsePacket(packet, servoCalibrationPacket)) return 2;
    readServoCalibrationPacket(servoCalibrationPacket, robotIO);
  } else if (packetType == PING) {
    readPingPacket(packet);
  } else {
    Serial.println("Unknown packet type!");
    return 1;
  }

  return 0;
}