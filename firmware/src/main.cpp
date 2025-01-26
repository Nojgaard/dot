#include <Arduino.h>
#include <Comm.h>
#include <SPI.h>
#include <ServoControllerPca9685.h>
#include <ServoControllerDirect.h>
#include <BatterySensor.h>

Comm comm;
ServoControllerPca9685 crtlServo;
BatterySensor batSensor;
TelemetryPacket telemetry;

void setup() {
  Serial.begin(9600);
  comm.begin();
  crtlServo.begin();
  batSensor.begin();
}

void updateSensorState() {
  telemetry.batteryVoltage = batSensor.readVoltage();
  telemetry.batteryCurrent = batSensor.readCurrent();
  Serial.println(telemetry.batteryVoltage);
  Serial.println(telemetry.batteryCurrent);
}

void loop() {
  if (comm.isTimedOut()) {
    Serial.println("Comms timed out! Detaching servos...");
    crtlServo.detach();
    updateSensorState();
    delay(500);
    return;
  }

  updateSensorState();
  comm.sendSensorState(telemetry);

  if (comm.isServoPacketAvailable()) {
    Serial.println("Received servo packet!");
    const ServoPacket& servoPacket = comm.consumeServoPacket();
    crtlServo.writeMicroSeconds(servoPacket.microseconds);
  }
}
