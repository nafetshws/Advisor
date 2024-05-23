#include <Arduino.h>
#include "../../include/motor.hpp"
#include "../../include/encoder.hpp"
#include "../../include/tof.hpp"


// ESP32 PIN OUT ////////////////////////////////

// PINS MOTOR A
#define MOTORA_IN1  0
#define MOTORA_IN2  2
#define MOTORA_PWM  15
#define MOTORA_ENCODER1 36
#define MOTORA_ENCODER2 39
// PINS MOTOR B
#define MOTORB_IN1  4
#define MOTORB_IN2  16
#define MOTORB_PWM  17
#define MOTORB_ENCODER1 35
#define MOTORB_ENCODER2 34

// PINS TOF SENSORS
#define TOF1_SHT_PIN 32
#define TOF2_SHT_PIN 33
#define TOF3_SHT_PIN 25
#define TOF4_SHT_PIN 26
#define TOF5_SHT_PIN 27
#define TOF6_SHT_PIN 14


// Global Motor Objects
Motor motorA = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
Motor motorB = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

// TOF Sensor Objects
TOF tofLeft =         TOF(1, TOF_START_ADDR + 0, TOF1_SHT_PIN);
TOF tofLeftFront =    TOF(2, TOF_START_ADDR + 1, TOF2_SHT_PIN);
TOF tofRightFront =   TOF(3, TOF_START_ADDR + 2, TOF3_SHT_PIN);
TOF tofRight =        TOF(4, TOF_START_ADDR + 3, TOF4_SHT_PIN);
TOF tofLeft45 =       TOF(5, TOF_START_ADDR + 4, TOF5_SHT_PIN);
TOF tofRight45 =      TOF(6, TOF_START_ADDR + 5, TOF6_SHT_PIN);





void setup() {

  // SETUP SERIAL MONITOR ///////////////////////

  // start serial monitor
  Serial.begin(115200);
  Serial.println("\nSETUP: Serial Monitor running");



  // SETUP MOTORS ///////////////////////////////

  // Initialise the motor objects, start pwm channels, confiure pins
  motorA.initialise();
  motorB.initialise();
  // Initialise the motor encoders pins and interrupts, init timer interrupt
  initEncoders(MOTORA_ENCODER1, MOTORA_ENCODER2, MOTORB_ENCODER1, MOTORB_ENCODER2);

  Serial.println("SETUP: Motor initialised");



  // SETUP TOF //////////////////////////////////

  Serial.println("SETUP: Try to connect to TOF sensors...");

  // init all the tof sensors
  initTofSensors(tofLeft, tofLeftFront, tofRightFront, tofRight, tofLeft45, tofRight45);

  Serial.println("SETUP: TOF Sensors initialised");

  // SETUP END //////////////////////////////////
  Serial.println("SETUP: Setup Done");
  Serial.println("Start Looping:");
}

void loop() {

  // Switch motor A on with Speed 100 (turns onboard led on, same pin motor)
  motorA.turnForward(100);
  motorB.turnForward(100);
  delay(3000);

  // print the rps of motor As
  Serial.printf("Motor A Umdrehung/sek: %d\n", getDirMotorA());
  Serial.printf("Motor B Umdrehung/sek: %d\n", getDirMotorB());

    // Switch motor A on with Speed 100 (turns onboard led on, same pin motor)
  motorA.turnBackward(50);
  motorB.turnBackward(50);
  delay(3000);

  // print the rps of motor As
  Serial.printf("Motor A Umdrehung/sek: %d\n", getDirMotorA());
  Serial.printf("Motor B Umdrehung/sek: %d\n", getDirMotorB());

  // Stop motor A (turns onboard led of, same pin as motor)
  motorA.stopMotor();
  motorB.stopMotor();
  delay(5000);


}