#include <Debug.h>
#include <IMU.h>
#include <Preferences.h>

#include "Wire.h"

static const float GRAVITY = 9.8006f;
VectorFloat gravity;
int16_t imuCalibrationOffsets[6] = {0, 0, 0, 0, 0, 0};
Preferences prefs;

bool IMU::begin() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having
                          // compilation difficulties

  // initialize device
  DEBUG_PRINTLN(F("Initializing I2C devices..."));
  _mpu.initialize();

  // verify connection
  DEBUG_PRINTLN(F("Testing device connections..."));
  DEBUG_PRINTLN(_mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  if (!_mpu.testConnection()) {
    _deviceStatus = 3;
    return false;
  }

  // load and configure the DMP
  DEBUG_PRINTLN(F("Initializing DMP..."));
  _deviceStatus = _mpu.dmpInitialize();  // 0 if successful

  if (_deviceStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    DEBUG_PRINT("DMP Initialization failed (code ");
    DEBUG_PRINT(_deviceStatus);
    DEBUG_PRINTLN(")");
    return false;
  }

  // read the calibration from EEPROM
  prefs.begin("IMU");
  readCalibration();

  // turn on the DMP, now that it's ready
  DEBUG_PRINTLN((F("Enabling DMP...")));
  _mpu.setDMPEnabled(true);

  // get expected DMP packet size for later comparison
  _packetSize = _mpu.dmpGetFIFOPacketSize();

  return true;
}

uint8_t IMU::read(VectorFloat& yawPitchRoll, VectorFloat& acceleration) {
  if (_deviceStatus != 0) return 1;

  // dmpGetCurrentFIFOPacket return 1 on success and 0 on failure
  uint8_t success = _mpu.dmpGetCurrentFIFOPacket(_fifoBuffer);

  if (success == 0) return success;

  success = 0;
  // read yaw pitch and roll
  success += _mpu.dmpGetQuaternion(&_orientation, _fifoBuffer);
  success += _mpu.dmpGetGravity(&_gravity, &_orientation);
  success += _mpu.dmpGetYawPitchRoll(&yawPitchRoll.x, &_orientation, &_gravity);

  float tmp = yawPitchRoll.x;
  yawPitchRoll.x = yawPitchRoll.z;
  yawPitchRoll.z = tmp;

  // read acceleration in inertial frame
  success += _mpu.dmpGetAccel(&_accelerationSensor, _fifoBuffer);
  success += _mpu.dmpGetLinearAccel(&_accelerationReal, &_accelerationSensor,
                                    &_gravity);
  success += _mpu.dmpConvertToWorldFrame(&_inertialFrameAcceleration,
                                         &_accelerationReal, &_orientation);

  acceleration.x = (_inertialFrameAcceleration.x / ACC_SCALE) * GRAVITY;
  acceleration.y = (_inertialFrameAcceleration.y / ACC_SCALE) * GRAVITY;
  acceleration.z = (_inertialFrameAcceleration.z / ACC_SCALE) * GRAVITY;

  return success;
}

void IMU::calibrate() {
  Serial.println("Old active offset:");
  _mpu.PrintActiveOffsets();
  // Calibration Time: generate offsets and calibrate our MPU6050
  Serial.println("Calibrating acceleration offsets...");
  _mpu.CalibrateAccel(10);
  Serial.println("");
  Serial.println("Calibrating gyro offsets...");
  _mpu.CalibrateGyro(10);
  Serial.println("");
  Serial.println("Done! Found new offsets:");
  _mpu.PrintActiveOffsets();
  Serial.println("Storing offsets on EEPROM...");
  storeCalibration();
  Serial.println("Calibration finished!");
}

void IMU::storeCalibration() {
  int address = 0;
  imuCalibrationOffsets[0] = _mpu.getXAccelOffset();
  imuCalibrationOffsets[1] = _mpu.getYAccelOffset();
  imuCalibrationOffsets[2] = _mpu.getZAccelOffset();
  imuCalibrationOffsets[3] = _mpu.getXGyroOffset();
  imuCalibrationOffsets[4] = _mpu.getYGyroOffset();
  imuCalibrationOffsets[5] = _mpu.getZGyroOffset();
  prefs.putBytes("IMU", (byte*)imuCalibrationOffsets,
                 sizeof(imuCalibrationOffsets));
}

void IMU::readCalibration() {
  prefs.getBytes("IMU", (byte*)imuCalibrationOffsets,
                 sizeof(imuCalibrationOffsets));

  _mpu.setXAccelOffset(imuCalibrationOffsets[0]);
  _mpu.setYAccelOffset(imuCalibrationOffsets[1]);
  _mpu.setZAccelOffset(imuCalibrationOffsets[2]);

  _mpu.setXGyroOffset(imuCalibrationOffsets[3]);
  _mpu.setYGyroOffset(imuCalibrationOffsets[4]);
  _mpu.setZGyroOffset(imuCalibrationOffsets[5]);
}

uint8_t IMU::deviceStatus() const { return _deviceStatus; }