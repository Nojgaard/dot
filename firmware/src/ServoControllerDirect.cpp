#include <ServoControllerDirect.h>

int servoPins[Specs::NUM_SERVOS] = {32, 33, 25, 26, 27, 14,
                                    12, 13, 19, 18, 5,  4};

#define USMIN 400
#define USMAX 2700

void ServoControllerDirect::begin() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 12; i++) {
    servos[i].setPeriodHertz(50);
  }
}

void ServoControllerDirect::writeMicroSeconds(int servoNum, int us) {
  if (us == 0) {
    servos[servoNum].detach();
  }

  if (!servos[servoNum].attached()) {
    servos[servoNum].attach(servoPins[servoNum]);
  }

  us = constrain(us, USMIN, USMAX);
  servos[servoNum].writeMicroseconds(us);
}

void ServoControllerDirect::writeMicroSeconds(const int us[Specs::NUM_SERVOS]) {
  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    writeMicroSeconds(i, us[i]);
  }
}

void ServoControllerDirect::detach() {
  for (int i = 0; i < Specs::NUM_SERVOS; i++){
    servos[i].detach();
  }
}