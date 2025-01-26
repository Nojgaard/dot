#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include <comm.h>

const char *ssid = "Telenor7748bod";
const char *password = "77xugJACB";

IPAddress remoteIP = INADDR_NONE;
IPAddress localIP(10, 0, 0, 88);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);

const int COMM_PORT = 9999;
const unsigned long SENSOR_DELAY_MS = 33;

bool Comm::begin() {
  lastSentSensorPacketMs = 0;
  if (!connectToWifi()) {
    return false;
  }

  if (!connectToController()) {
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

bool Comm::connectToController() {
  Serial.println("Connecting to controller");

  if (!udp.listen(COMM_PORT)) {
    Serial.println("Could not listen on selected port");
    return false;
  }

  setupPacketReceiver();

  while (!isConnected()) {
    Serial.print(".");
    delay(2000);
  }

  return true;
}

bool Comm::isConnected() { return remoteIP != INADDR_NONE; }

bool Comm::isServoPacketAvailable() { return hasServoPacket; }

bool Comm::isTimedOut() {
  return (millis() - lastReceivedPacketMs) > timeoutMs;
}

const ServoPacket &Comm::consumeServoPacket() {
  hasServoPacket = false;
  return lastServoPacket;
}

void Comm::setupPacketReceiver() {
  udp.onPacket([this](AsyncUDPPacket &packet) {
    lastReceivedPacketMs = millis();
    if (!isConnected()) {
      remoteIP = packet.remoteIP();
      Serial.print("Found controller with IP:");
      Serial.println(remoteIP);
    }

    if (packet.length() != sizeof(ServoPacket)) {
      Serial.println("Received ping packet");
      Serial.println(packet.remoteIP());
      Serial.println(packet.remotePort());
      Serial.println(packet.localIP());
      Serial.println(packet.localPort());
      packet.write(0);
      return;
    }

    hasServoPacket = true;
    memcpy(&lastServoPacket, packet.data(), sizeof(ServoPacket));
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