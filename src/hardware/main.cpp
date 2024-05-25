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

// PINS TOF/IR SENSORS
#define SENSOR_SLOT_1 32    // IR
#define SENSOR_SLOT_2 33    // IR
#define SENSOR_SLOT_3 25    // TOF
#define SENSOR_SLOT_4 26    // TOF
#define SENSOR_SLOT_5 27    // empty
#define SENSOR_SLOT_6 14    // emtpy



// GLOBAL OBJECTS ///////////////////////////////

// Global Motor Objects
Motor motorA = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
Motor motorB = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

// TOF Sensor Objects
TOF tofLeftFront =   TOF(1, TOF_START_ADDR + 1, SENSOR_SLOT_3);
TOF tofRightFront =  TOF(2, TOF_START_ADDR + 2, SENSOR_SLOT_4);
// TOF tofLeft =        TOF(3, TOF_START_ADDR + 3, SENSOR_SLOT_5);
// TOF tofRight =       TOF(4, TOF_START_ADDR + 4, SENSOR_SLOT_6);




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
  // initTofSensors(tofLeft, tofLeftFront, tofRightFront, tofRight);
  initTofSensors(tofLeftFront, tofRightFront);

  Serial.println("SETUP: TOF Sensors initialised");

  // SETUP END //////////////////////////////////
  Serial.println("SETUP: Setup Done");
  Serial.println("Start Looping:");
}




void loop() {

  // Read out Tof Sensor data
  delay(1000);

}