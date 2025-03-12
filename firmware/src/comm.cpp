#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include <comm.h>
#

const char *ssid = "Telenor7748bod";
const char *password = "77xugJACB";

IPAddress remoteIP = INADDR_NONE;
IPAddress localIP(10, 0, 0, 88);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);

const int COMM_PORT = 9999;
const unsigned long SENSOR_DELAY_MS = 33;

bool Comm::begin(RobotIO& robotIO) {
  lastSentSensorPacketMs = 0;
  if (!connectToWifi()) {
    return false;
  }

  if (!connectToController(robotIO)) {
    return false;
  }

  return true;
}

bool Comm::connectToWifi() {
  Serial.println("Connecting to wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    return false;
  }

  if (!WiFi.config(localIP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
    return false;
  }

  Serial.print("Connected to wifi with local IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

bool Comm::connectToController(RobotIO& robotIO) {
  Serial.println("Connecting to controller");

  if (!udp.listen(COMM_PORT)) {
    Serial.println("Could not listen on selected port");
    return false;
  }

  setupPacketReceiver(robotIO);

  while (!isConnected()) {
    Serial.print(".");
    delay(2000);
  }

  return true;
}

bool Comm::isConnected() { return remoteIP != INADDR_NONE; }

bool Comm::isTimedOut() {
  return (millis() - lastReceivedPacketMs) > timeoutMs;
}

void Comm::setupPacketReceiver(RobotIO& robotIO) {
  udp.onPacket([&](AsyncUDPPacket &packet) {
    this->lastReceivedPacketMs = millis();
    if (!isConnected()) {
      remoteIP = packet.remoteIP();
      Serial.print("Found controller with IP:");
      Serial.println(remoteIP);
    }

    readPacket(packet, robotIO);
  });
}

void Comm::sendSensorState(const TelemetryPacket &packet) {
  if (!isConnected()) {
    return;
  }

  unsigned long currentTimeMs = millis();
  if ((currentTimeMs - lastSentSensorPacketMs) < SENSOR_DELAY_MS) {
    return;
  }

  lastSentSensorPacketMs = currentTimeMs;

  size_t res = udp.writeTo((uint8_t *)&packet, sizeof(TelemetryPacket), remoteIP, COMM_PORT);
}