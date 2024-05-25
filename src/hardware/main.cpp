#include <Arduino.h>
#include "../../include/motor.hpp"
#include "../../include/encoder.hpp"
#include "../../include/tof.hpp"
#include "../../include/ir.hpp"

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

// DIP Switches
#define DIP_SWITCH_PIN_1 13
#define DIP_SWITCH_PIN_2 12


// Global Motor Objects
Motor motorA = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
Motor motorB = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

// TOF Sensor Objects
TOF tofLeftFront =    TOF(2, TOF_START_ADDR + 1, TOF3_SHT_PIN);
TOF tofRightFront =   TOF(3, TOF_START_ADDR + 2, TOF4_SHT_PIN);

// IR Sensor objects
IR leftIR = IR (TOF1_SHT_PIN);
IR rightIR = IR (TOF1_SHT_PIN);
int turnTime = 270;
int speed = 80;

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
  initTofSensors(tofLeftFront, tofRightFront);

  Serial.println("SETUP: TOF Sensors initialised");

  // SETUP DIP switches /////////////////////////
  pinMode(DIP_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(DIP_SWITCH_PIN_2, INPUT_PULLUP);

  // SETUP END //////////////////////////////////
  Serial.println("SETUP: Setup Done");
  Serial.println("Start Looping:");
}

bool checkForStartSignal() {
  int switch1 = digitalRead(DIP_SWITCH_PIN_1) == 0 ? 1 : 0;
  int switch2 = digitalRead(DIP_SWITCH_PIN_2) == 0 ? 1 : 0;

  Serial.printf("DIP1: %d\t DIP2: %d\n", switch1, switch2);

  return switch1 == HIGH && switch2 == HIGH; 
}

void turnRight() {
  Serial.printf("*********************\n");
  Serial.printf("Start turning\n");
  Serial.printf("*********************\n");
  long startTime = millis();

  while (millis() - startTime < turnTime) {
    motorA.turnBackward(speed);
    motorB.turnForward(speed);
  }

  motorA.turnForward(0);
  motorB.turnForward(0);
}

void turnLeft() {
  Serial.printf("*********************\n");
  Serial.printf("Start turning\n");
  Serial.printf("*********************\n");
  long startTime = millis();

  while (millis() - startTime < turnTime) {
    motorA.turnForward(speed);
    motorB.turnBackward(speed);
  }

  motorA.turnForward(0);
  motorB.turnForward(0);
}

bool checkForTurnSignal() {
  Serial.printf("Checking for turn signal\n");
  int switch1 = digitalRead(DIP_SWITCH_PIN_1) == 0 ? 1 : 0;
  int switch2 = digitalRead(DIP_SWITCH_PIN_2) == 0 ? 1 : 0;


  return (switch1 == LOW && switch2 == HIGH); 
}

void driveTillObstacle() {
  uint16_t max_distance = 85;

  uint16_t topLeftDistance = tofLeftFront.getDist();
  uint16_t topRightDistance = tofRightFront.getDist();
  uint16_t irLeftDistance = leftIR.isTriggered();
  uint16_t irRightDistance = rightIR.isTriggered();

  motorA.turnForward(50);
  motorB.turnForward(50);
  
  Serial.printf("TOF top left: %d\t TOF top rigth: %d\n", topLeftDistance, topRightDistance);

  while (topLeftDistance > max_distance && topRightDistance > max_distance) {
    topLeftDistance = tofLeftFront.getDist();
    topRightDistance = tofRightFront.getDist();
    irLeftDistance = leftIR.isTriggered();
    irRightDistance = rightIR.isTriggered();

    Serial.printf("TOF top left: %d\t TOF top rigth: %d\n", topLeftDistance, topRightDistance);
  }

  motorA.turnForward(0);
  motorB.turnForward(0);

  // while (1) {
  //   Serial.printf("LOOP EXITING\n");
  // }
}

void loop() {
  if (checkForStartSignal()) {
    delay(5 * 1000);
    driveTillObstacle();
    delay(2 * 1000);
    turnRight();
    delay(2 * 1000);
    driveTillObstacle();
    delay(2 * 1000);
    turnRight();
    delay(2 * 1000);
    driveTillObstacle();
    delay(2 * 1000);
    turnRight();
    delay(2 * 1000);
    driveTillObstacle();
    while (1) {
      delay(2 * 1000);
      turnRight();
      delay(2 * 1000);
      driveTillObstacle();
    }

    // driveTillObstacle();
    // turnRight();
    while (1) {
      Serial.printf("LOOPING EXITING\n");
    }
  }
  return;
// if (checkForTurnSignal()) {
//       Serial.printf("About to turn\n");
//       delay(5000);
//       turnRight();
//       isDriving = false;
//       return;
//   } else if (!checkForStartSignal()) {
//     motorA.turnForward(0);
//     motorB.turnForward(0);
//     delay(1000);
//     return;
//   } else {
//     if (!isDriving) delay(10 * 1000);
//   }

//   isDriving = true;

//   motorA.turnForward(50);
//   motorB.turnForward(50);

//   uint16_t topLeftDistance = tofLeftFront.getDist();
//   uint16_t topRightDistance = tofRightFront.getDist();
//   uint16_t irLeftDistance = leftIR.isTriggered();
//   uint16_t irRightDistance = rightIR.isTriggered();
  
//   Serial.printf("TOF top left: %d\t TOF top rigth: %d\n", topLeftDistance, topRightDistance);

//   if (topLeftDistance <= 80 || topRightDistance <= 80) {
//     motorA.turnForward(0);
//     motorB.turnForward(0);
//     isDriving = false;
//     while (1) {
//       Serial.printf("LOOP EXITING\n");
//     }
//   }
}