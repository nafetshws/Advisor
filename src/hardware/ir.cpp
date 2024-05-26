#include "../../include/ir.hpp"
#include <Arduino.h>

IR::IR() {}

IR::IR (uint8_t IR_SENSOR_PIN) {
    this->IR_SENSOR_PIN = IR_SENSOR_PIN;
    pinMode(IR_SENSOR_PIN, INPUT);
}

bool IR::isTriggered() {
    int sensorStatus = digitalRead(this->IR_SENSOR_PIN);
    // HIGH when noting is detected, else LOW (this method returns the reverse)
    return sensorStatus != 1;
}