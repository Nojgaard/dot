#include <Arduino.h>
#include <comm.h>

Comm socket;

void setup()
{
  Serial.begin(9600);
  socket.begin();
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println("Waiting for packets....");
  delay(5000);
}
