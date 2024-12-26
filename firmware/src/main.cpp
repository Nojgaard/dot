#include <Arduino.h>
#include <Comm.h>
#include <SPI.h>
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

    Serial.println("Received servo packet!");
    for (int i = 0; i < Specs::NUM_SERVOS; i++) {
      Serial.print(servoPacket.microseconds[i]);
      Serial.print(" ");
    }
    Serial.println();

    crtlServo.writeMicroSeconds(servoPacket.microseconds);
  }
}
