#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include <comm.h>

const char *ssid = "Telenor7748bod";
const char *password = "77xugJACB";

IPAddress remoteIP(0, 0, 0, 0);
IPAddress localIP(10, 0, 0, 88);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);

bool Comm::begin() {
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

  if (!udp.listen(9999)) {
    Serial.println("Could not listen on selected port");
    return false;
  }

  bool foundController = false;

  udp.onPacket([&foundController](AsyncUDPPacket packet) {
    if (packet.length() != 4) {
      Serial.println("Received packet of unexpected size");
      return;
    }

    uint8_t* octets = packet.data();
    remoteIP = IPAddress(octets[0], octets[1], octets[2], octets[3]);
    foundController = true;

    Serial.print("Found controller with IP:");
    Serial.println(remoteIP);

    packet.write(0);
  });

  while (!foundController) {
    Serial.print(".");
    delay(2000);
  }

  return true;
}