#include "../../include/motor.hpp"
#include <Arduino.h>

Motor::Motor() {
    //Empty Constrcutor
}

Motor::Motor (uint8_t pinA, uint8_t pinB, uint8_t pinPWM, uint8_t pwmChannel) {
    this->pinA = pinA;
    this->pinB = pinB;
    this->pinPWM = pinPWM;
    this->pwmChannel = pwmChannel;
}

void Motor::initialise() {
    // Configure PWM Channels
    ledcSetup(pwmChannel, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);

    // Attach Pins to the PWM Channels
    ledcAttachPin(pinPWM, pwmChannel);

    // Confiure the diretion pins as output
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
}


void Motor::turnForward(uint16_t speed) {
    // set direction forward
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    // set speed with pwm pin over pwm channel
    if (pwmChannel == MOTORA_PWM_CHANNEL) {
        //right motor needs to be 5% faster
        ledcWrite(pwmChannel, 1.05 * speed);
    } else {
        ledcWrite(pwmChannel, speed);
    }
    // save speed
    this->speed = speed;
}


void Motor::turnBackward(uint16_t speed) {
    // set direction backward
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    // set speed with pwm pin over pwm channel
    if (pwmChannel == MOTORA_PWM_CHANNEL) {
        //right motor needs to be 5% faster
        ledcWrite(pwmChannel, 1.05 * speed);
    } else {
        ledcWrite(pwmChannel, speed);
    }
    // save speed
    this->speed = speed;
}

uint16_t Motor::getSpeed() {
    return this->speed;
}

void Motor::stopMotor() {
    // TODO: Don't stop motor instantanious

    // disable direction
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    // set speed to 0
    ledcWrite(pwmChannel, 0);
    // save speed
    this->speed = 0;
}