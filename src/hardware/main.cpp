#include <Arduino.h>
#include "../../include/motor.hpp"


// ESP32 PIN OUT ////////////////////////////////

// PINS MOTOR A
#define MOTORA_IN1  0
#define MOTORA_IN2  2
#define MOTORA_PWM  15
// PINS MOTOR B
#define MOTORB_IN1  4
#define MOTORB_IN2  16
#define MOTORB_PWM  17


// Global Motor Objects
Motor motorA = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
Motor motorB = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);



void setup() {

  // SETUP SERIAL MONITOR ///////////////////////

  // start serial monitor
  Serial.begin(115200);
  Serial.println("\nSETUP: Serial Monitor running");


  // SETUP MOTORS ///////////////////////////////

  // Initialise the motor objects, start pwm channels, confiure pins
  motorA.initialise();
  motorB.initialise();
  Serial.println("SETUP: Motor initialised");

  // SETUP END //////////////////////////////////
  Serial.println("SETUP: Setup Done");
  Serial.println("Start Looping:");
}

void loop() {

  // Switch motor A repeatly on and off (Onboard LED should also blink, uses same pin)
  motorA.turnBackward(100);
  delay(5000);
  motorA.stopMotor();
  delay(5000);

}