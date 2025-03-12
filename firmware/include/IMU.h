#pragma once

#include "MPU6050_6Axis_MotionApps20.h"

/**
 * Inertial Measuring Unit (IMU) used to retrieve the euler angles and acceleration
 * of the quadcopter during flight.
 * 
 * The IMU uses the Digital Motion Processor (DMP) onboard the Motion Processing Unit (MPU)
 * MPU6050 to obtain accurate readings.
*/
class IMU {
    public:
    bool begin();
    uint8_t read(VectorFloat& yawPitchRoll, VectorFloat& acceleration);
    void calibrate();
    uint8_t deviceStatus() const;
    //const VectorFloat& orientation();
    //const VectorFloat& acceleration();

    private:
    void storeCalibration();
    void readCalibration();

    MPU6050 _mpu;
    uint8_t _deviceStatus;
    //VectorFloat _orientation;
    //VectorFloat _acceleration;

    // Acceleration 2G scale
    float ACC_SCALE = 16384 / 2.0;

    // MPU control/status vars
    uint16_t _packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t _fifoCount;     // count of all bytes currently in FIFO
    uint8_t _fifoBuffer[64]; // FIFO storage buffer

    Quaternion _orientation;               // [w, x, y, z]         quaternion container
    VectorInt16 _accelerationSensor;       // [x, y, z]            accel sensor measurements
    VectorInt16 _accelerationReal;         // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 _inertialFrameAcceleration; // [x, y, z]           gravity-free accel sensor measurements in inertial frame
    VectorFloat _gravity;                  // [x, y, z]            gravity vector
};