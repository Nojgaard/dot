#include <ServoControllerDirect.h>

int servo1Pin = 14;

void ServoControllerDirect::begin() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 12; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servo1Pin, 544, 2500);
    break;
  }
}