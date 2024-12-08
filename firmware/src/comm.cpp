#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include <comm.h>

const char *ssid = "Telenor7748bod";
const char *password = "77xugJACB";

IPAddress localIP(10, 0, 0, 88);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);

bool Comm::begin() {
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
  
  if (udp.listen(9999)) {
    Serial.println("UDP server started");

    // Callback for incoming packets
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();

      // Echo the message back to the sender
      packet.printf("Echo: %s", (char *)packet.data());
    });
  }
}