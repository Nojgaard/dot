#include <BatterySensor.h>
#include <Arduino.h>

const int SENSOR_PIN = 34;
const float MAX_BAT_V = 8.4;
const float MIN_BAT_V = 7.0;
const float REF_V = 3.3;
const float ADC_RANGE = 4096;
const float R1_VOLTAGE_DIVIDER = 100000;
const float R2_VOLTAGE_DIVIDER = 47000;

void BatterySensor::begin() {
}

float BatterySensor::readVoltage() {
    //float voltage = (adc1_get_raw(SENSOR_CHANNEL) * REF_V) / ADC_RANGE;
    float voltage = (analogRead(SENSOR_PIN) * REF_V) / ADC_RANGE;    
    voltage /= R2_VOLTAGE_DIVIDER / (R1_VOLTAGE_DIVIDER + R2_VOLTAGE_DIVIDER);
    return voltage;
}

int BatterySensor::readPercentage() {
    float voltage = readVoltage();
    float voltagePercent = ((voltage - MIN_BAT_V) / (MAX_BAT_V - MIN_BAT_V)) * 100;
    return (int)roundf(voltagePercent); 
}
