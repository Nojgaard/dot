#include <ServoControllerPca9685.h>
#include <Arduino.h>

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define USMIN 400
#define USMAX 2700
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

void ServoControllerPca9685::begin() {
  pwm.begin();

  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency.
   * Failure to correctly set the int.osc value will cause unexpected PWM
   * results
   */
  // pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void ServoControllerPca9685::writeMicroSeconds(int servoNum, int us) {
  if (us == 0) {  // disable servo
    pwm.setPin(servoNum, 0);
  } else {
    us = constrain(us, USMIN, USMAX);
    pwm.writeMicroseconds(servoNum, us);
  }
}

void ServoControllerPca9685::writeMicroSeconds(const int us[Specs::NUM_SERVOS]) {
  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    writeMicroSeconds(i, us[i]);
  }
}

void ServoControllerPca9685::detach() {
  for (int i = 0; i < Specs::NUM_SERVOS; i++) {
    writeMicroSeconds(i, 0);
  }
}