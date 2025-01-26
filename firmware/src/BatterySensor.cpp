#include <BatterySensor.h>
#include <Arduino.h>

const int V_SENSOR_PIN = 34;
const int A_SENSOR_PIN = 35;
const float MAX_BAT_V = 8.4;
const float MIN_BAT_V = 7.0;
const float REF_V = 3.3;
const float ADC_RANGE = 4096;
const float R1_VOLTAGE_DIVIDER = 100000;
const float R2_VOLTAGE_DIVIDER = 47000;

float readPinVoltage(float pin) {
    return (analogRead(pin) * REF_V) / ADC_RANGE;
}

void BatterySensor::begin() {
    /*adcAttachPin(V_SENSOR_PIN);
    adcAttachPin(A_SENSOR_PIN);
    analogReadResolution(12);
    analogSetWidth(12);
    analogSetAttenuation(ADC_11db);*/
}

float BatterySensor::readVoltage() {
    float voltage = readPinVoltage(V_SENSOR_PIN);    
    voltage /= R2_VOLTAGE_DIVIDER / (R1_VOLTAGE_DIVIDER + R2_VOLTAGE_DIVIDER);
    return voltage;
}

int BatterySensor::readPercentage() {
    float voltage = readVoltage();
    float voltagePercent = ((voltage - MIN_BAT_V) / (MAX_BAT_V - MIN_BAT_V)) * 100;
    return (int)roundf(voltagePercent); 
}

float BatterySensor::readCurrent() {
    float vPerA = 0.066; // 30A mode
    float bias = 2.0;
    return (2.5 - readPinVoltage(A_SENSOR_PIN)) / vPerA - bias;
}
