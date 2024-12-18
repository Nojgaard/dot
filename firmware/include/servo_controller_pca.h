#pragma once

#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

class ServoControllerPca {
    public:
    void begin();
    void writeMicroSeconds(int servoNum, int us);
    private:
    Adafruit_PWMServoDriver pwm;
};