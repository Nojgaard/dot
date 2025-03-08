#include <RobotIO.h>

void RobotIO::begin() {
    servo.begin();
    imu.begin();
    battery.begin();
}