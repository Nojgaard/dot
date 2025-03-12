#include <RobotIO.h>

void RobotIO::begin() {
    servo.begin();
    imu.begin();
    battery.begin();
}

void printDeviceStatus(uint8_t deviceStatus) {
    if (deviceStatus == 0) {
        Serial.println("OK");
    } else {
        Serial.print("ERROR [");
        Serial.print(deviceStatus);
        Serial.println("]");
    }
}

void RobotIO::printStatus() const {
    Serial.println("Device status:");
    Serial.print("  IMU: ");
    printDeviceStatus(imu.deviceStatus());
    Serial.print("  Servo: ");
    printDeviceStatus(servo.deviceStatus());
}