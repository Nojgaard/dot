#include <Arduino.h>
#include <ESP32Servo.h>
#include <comm.h>
#include <SPI.h>
#include <servo_controller_pca.h>

Comm socket;
Servo servo;
int servo1Pin = 14;
ServoControllerPca crtlServo;

void setup() {
  Serial.begin(9600);
  // socket.begin();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);
  servo.attach(servo1Pin, 544, 2500);

  crtlServo.begin();
}

void loop() {
  int minUs = 800;

  Serial.println("Writing 544");
  servo.write(0);
  crtlServo.writeMicroSeconds(0, 500);
  delay(5000);
  servo.write(180);
  Serial.println("Writing 2500");
  crtlServo.writeMicroSeconds(0, 2500);

  // servo.detach();
  //  put your main code here, to run repeatedly:
  //  Serial.println("Waiting for packets....");
  delay(5000);
}
