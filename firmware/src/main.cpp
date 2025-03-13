#include <Arduino.h>
#include <Comm.h>
#include <SPI.h>
#include <RobotIO.h>

Comm comm;
RobotIO robotIO;
TelemetryPacket telemetry;
long lastLoopMs;

void setup() {
  Serial.begin(9600);

  Serial.println("Setting up robot devices...");
  robotIO.begin();
  robotIO.printStatus();

  Serial.println("Setting up communications...");
  comm.begin(robotIO);
  lastLoopMs = millis();
}

void updateSensorState() {
  telemetry.batteryVoltage = robotIO.battery.readVoltage();
  telemetry.batteryCurrent = robotIO.battery.readCurrent();
  robotIO.imu.read(telemetry.orientation, telemetry.acceleration);
  telemetry.statusIMU = robotIO.imu.deviceStatus();
  telemetry.statusServo = robotIO.servo.deviceStatus();
  //Serial.println(telemetry.batteryVoltage);
  //Serial.println(telemetry.batteryCurrent);
}

void loop() {
  long currentLoopMs = millis();
  float dt = (currentLoopMs - lastLoopMs) / 1000.0;
  long dtMillis = (currentLoopMs - lastLoopMs);
  lastLoopMs = currentLoopMs;

  updateSensorState();
  comm.sendSensorState(telemetry);

  if (comm.isTimedOut()) {
    Serial.println("Comms timed out! Detaching servos...");
    robotIO.servo.detach();
    delay(500);
    return;
  }

  robotIO.servo.actuate(dt);

}
