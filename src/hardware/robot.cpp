#include "../include/robot.hpp"
#include <cstdint>

Robot::Robot() {
  this->motorA = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
  this->motorB = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

  // IR Sensor objects
  this->leftIR = IR (TOF1_SHT_PIN);
  this->rightIR = IR (TOF1_SHT_PIN);

  this->turnTime = 270;
  this->turnSpeed = 80;
  this->driveSpeed = 50;
}

void Robot::setupRobot() {
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
}

void Robot::driveTillObstacle() {
  uint16_t max_distance = 85;

  uint16_t topLeftDistance  = this->tofLeftFront.getDist();
  uint16_t topRightDistance = this->tofRightFront.getDist();
  uint16_t irLeftDistance   = this->leftIR.isTriggered();
  uint16_t irRightDistance  = this->rightIR.isTriggered();

  motorA.turnForward(driveSpeed);
  motorB.turnForward(driveSpeed);
  
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
}

void Robot::turnRight() {
  long startTime = millis();

  while (millis() - startTime < turnTime) {
    motorA.turnBackward(turnSpeed);
    motorB.turnForward(turnSpeed);
  }

  motorA.turnForward(0);
  motorB.turnForward(0);
}

void Robot::turnLeft() {
  long startTime = millis();

  while (millis() - startTime < turnTime) {
    motorA.turnForward(turnSpeed);
    motorB.turnBackward(turnSpeed);
  }

  motorA.turnForward(0);
  motorB.turnForward(0);
}

bool Robot::checkForTurnSignal() {
  int switch1 = digitalRead(DIP_SWITCH_PIN_1) == 0 ? 1 : 0;
  int switch2 = digitalRead(DIP_SWITCH_PIN_2) == 0 ? 1 : 0;


  return (switch1 == LOW && switch2 == HIGH); 
}

bool Robot::checkForStartSignal() {
  int switch1 = digitalRead(DIP_SWITCH_PIN_1) == 0 ? 1 : 0;
  int switch2 = digitalRead(DIP_SWITCH_PIN_2) == 0 ? 1 : 0;

  Serial.printf("DIP1: %d\t DIP2: %d\n", switch1, switch2);

  return switch1 == HIGH && switch2 == HIGH; 
}

bool Robot::wallFront() {
    return false;
}

bool Robot::wallRight() {
    return false;
}

bool Robot::wallLeft() {
    return false;
}

void Robot::moveForward(int distance) {

}