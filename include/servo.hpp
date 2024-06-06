#ifndef SERVO_HPP
#define SERVO_HPP

#include <ESP32Servo.h>

#define SERVO_ANGLE_UP  132
#define SERVO_ANGLE_DOWN 5
#define SERVO_PWM_CHANNEL 1

// ESP32Servo Object
static Servo servo;

/**
 * @brief Init the servo motor, brings arms to up position
 * @param pin   pin of servo motor
*/
void setupServo(int pin);

/**
 * @brief rotates servo to up position
*/
void servoUp();

/**
 * @brief rotates servo to down position
*/
void servoDown();

#endif