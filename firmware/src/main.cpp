#include <Arduino.h>
#include <SPI.h>
#include <Comm.h>
#include <ServoControllerPca9685.h>

Comm comm;
ServoControllerPca9685 crtlServo;

void setup() {
  Serial.begin(9600);
  comm.begin();
  crtlServo.begin();
}

void loop() {
  if (comm.isTimedOut()) {
    Serial.println("Comms timed out! Detaching servos...");
    crtlServo.detach();
    delay(500);
    return;
  }

  if (comm.isServoPacketAvailable()) {
    const ServoPacket& servoPacket = comm.consumeServoPacket();
    crtlServo.writeMicroSeconds(servoPacket.microseconds);
  }
}
