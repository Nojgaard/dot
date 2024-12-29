#include <Arduino.h>
#include <Comm.h>
#include <SPI.h>
#include <ServoControllerPca9685.h>
#include <BatterySensor.h>

Comm comm;
ServoControllerPca9685 crtlServo;
BatterySensor batSensor;
SensorPacket state;

void setup() {
  Serial.begin(9600);
  comm.begin();
  crtlServo.begin();
  batSensor.begin();
}

void updateSensorState() {
  state.batteryVoltage = batSensor.readVoltage();
}

void loop() {
  if (comm.isTimedOut()) {
    Serial.println("Comms timed out! Detaching servos...");
    crtlServo.detach();
    delay(500);
    return;
  }

  updateSensorState();
  comm.sendSensorState(state);

  if (comm.isServoPacketAvailable()) {
    Serial.println("Received servo packet!");
    const ServoPacket& servoPacket = comm.consumeServoPacket();
    crtlServo.writeMicroSeconds(servoPacket.microseconds);
  }
}
