#include "../../include/servo.hpp"

void setupServo(int pin) {
    // set the servo pin
    servo.attach(pin);
    // servo in end up position
    servo.write(SERVO_ANGLE_UP);
    delay(400);
}

void servoUp() {
    for (int i = SERVO_ANGLE_DOWN; i <= SERVO_ANGLE_UP; i++) {
        servo.write(i);
        delay(6);
    }
}

void servoDown() {
    for (int i = SERVO_ANGLE_UP; i >= SERVO_ANGLE_DOWN; i--) {
        servo.write(i);
        delay(6);
    }
}