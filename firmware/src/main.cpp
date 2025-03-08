#include <Arduino.h>
#include <Comm.h>
#include <SPI.h>
#include <RobotIO.h>

Comm comm;
RobotIO robotIO;
TelemetryPacket telemetry;

void setup() {
  Serial.begin(9600);
  comm.begin();
  robotIO.begin();
}

void updateSensorState() {
  telemetry.batteryVoltage = robotIO.battery.readVoltage();
  telemetry.batteryCurrent = robotIO.battery.readCurrent();
  robotIO.imu.read(telemetry.orientation, telemetry.acceleration);
  //Serial.println(telemetry.batteryVoltage);
  //Serial.println(telemetry.batteryCurrent);
}

void loop() {
  if (comm.isTimedOut()) {
    Serial.println("Comms timed out! Detaching servos...");
    robotIO.servo.detach();
    updateSensorState();
    delay(500);
    return;
  }

  updateSensorState();
  comm.sendSensorState(telemetry);

  if (comm.isServoPacketAvailable()) {
    Serial.println("Received servo packet!");
    const ServoPacket& servoPacket = comm.consumeServoPacket();
    robotIO.servo.writeMicroSeconds(servoPacket.microseconds);
  }
}
