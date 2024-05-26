#include "../include/robot.hpp"
#include <cstdint>

Robot::Robot() {
  this->motorRight = Motor(MOTORA_IN1, MOTORA_IN2, MOTORA_PWM, MOTORA_PWM_CHANNEL);
  this->motorLeft = Motor(MOTORB_IN1, MOTORB_IN2, MOTORB_PWM, MOTORB_PWM_CHANNEL);

  // IR Sensor objects
  this->irLeft = IR (TOF1_SHT_PIN);
  this->irRight = IR (TOF1_SHT_PIN);

  this->turnTime = 270;
  this->turnSpeed = 80;
  this->driveSpeed = 50;
  this->wallDistance = 120;
  this->cellWidth = 160;
  this->tofTurnError = 10;
  this->maxDriveSpeed = 100;
}

void Robot::setupRobot() {
  // SETUP SERIAL MONITOR ///////////////////////

  // start serial monitor
  Serial.begin(115200);
  Serial.println("\nSETUP: Serial Monitor running");

  // SETUP MOTORS ///////////////////////////////

  // Initialise the motor objects, start pwm channels, confiure pins
  motorRight.initialise();
  motorLeft.initialise();
  // Initialise the motor encoders pins and interrupts, init timer interrupt
  initEncoders(MOTORA_ENCODER1, MOTORA_ENCODER2, MOTORB_ENCODER1, MOTORB_ENCODER2);

  Serial.println("SETUP: Motor initialised");


  // SETUP TOF //////////////////////////////////
  Serial.println("SETUP: Try to connect to TOF sensors...");

  // init all the tof sensors
  initTofSensors(tofLeftFront, tofRightFront, tofLeft, tofRight);

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
  uint16_t irLeftDistance   = this->irLeft.isTriggered();
  uint16_t irRightDistance  = this->irRight.isTriggered();

  motorRight.turnForward(driveSpeed);
  motorLeft.turnForward(driveSpeed);
  
  Serial.printf("TOF top left: %d\t TOF top rigth: %d\n", topLeftDistance, topRightDistance);

  while (topLeftDistance > max_distance && topRightDistance > max_distance) {
    topLeftDistance = tofLeftFront.getDist();
    topRightDistance = tofRightFront.getDist();
    irLeftDistance = irLeft.isTriggered();
    irRightDistance = irRight.isTriggered();

    Serial.printf("TOF top left: %d\t TOF top rigth: %d\n", topLeftDistance, topRightDistance);
  }

  motorRight.stopMotor();
  motorLeft.stopMotor();
}

void Robot::turnRight() {
  long startTime = millis();

  while (millis() - startTime < turnTime) {
    motorRight.turnBackward(turnSpeed);
    motorLeft.turnForward(turnSpeed);
  }

  motorRight.stopMotor();
  motorLeft.stopMotor();

  this->correctTurnError();
}

void Robot::turnLeft() {
  long startTime = millis();

  while (millis() - startTime < turnTime) {
    motorRight.turnForward(turnSpeed);
    motorLeft.turnBackward(turnSpeed);
  }

  motorLeft.stopMotor();
  motorRight.stopMotor();
}

void Robot::correctTurnError() {
  uint16_t currentDistanceLeft;
  uint16_t currentDistanceRight;
  int distanceDifference;

  do {
    currentDistanceLeft = tofLeftFront.getDist();
    currentDistanceRight = tofRightFront.getDist();
    distanceDifference = currentDistanceLeft - currentDistanceRight;

    if (distanceDifference < 0) {
      // Turn right
      uint16_t turnTimeTmp = this->turnTime;
      // rotation time is proportional to the distance difference between the tof sensors
      this->turnTime = abs(distanceDifference);
      this->turnRight();
      this->turnTime = turnTimeTmp;
    } else {
      // Turn left
      uint16_t turnTimeTmp = this->turnTime;
      // rotation time is proportional to the distance difference between the tof sensors
      this->turnTime = abs(distanceDifference);
      this->turnLeft();
      this->turnTime = turnTimeTmp;
    }

  } while (abs(distanceDifference) >= tofTurnError);
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
  return tofLeftFront.getDist() < this->wallDistance || tofRightFront.getDist() < this->wallDistance;
}

bool Robot::wallRight() {
  return tofRight.getDist() < this->wallDistance;
}

bool Robot::wallLeft() {
  return tofLeft.getDist() < this->wallDistance;
}

void Robot::moveForward(int distance) {
  uint16_t startDistanceLeft = tofLeftFront.getDist();
  uint16_t startDistanceRight = tofRightFront.getDist();
  uint16_t startDistance = (startDistanceLeft + startDistanceRight) / 2;

  motorRight.turnForward(this->driveSpeed);
  motorLeft.turnForward(this->driveSpeed);

  uint16_t currentDistanceLeft;
  uint16_t currentDistanceRight;
  uint16_t currentDistanceToWall;

  do {
    // Insert error correction /////
    //this->correctSteeringError();
    ////////////////////////////////
    currentDistanceLeft = tofLeftFront.getDist();
    currentDistanceRight = tofRightFront.getDist();
    currentDistanceToWall = (currentDistanceLeft + currentDistanceRight) / 2;
  } while (startDistance - (this->cellWidth * distance) <= currentDistanceToWall); 

  motorLeft.stopMotor();
  motorRight.stopMotor();
}

void Robot::correctSteeringError() {
  // bool isLeftTriggered = irLeft.isTriggered();
  // bool isRightTriggered = irRight.isTriggered();

  // if (isLeftTriggered && isRightTriggered) return;
  
  //

  float error = 0.00F;
  float prevError = 0.00F;
  float prevIterm = 0;
  const uint8_t kp  = 0; // Tune (Proportional Constant)
  const uint8_t ki = 0; // Tune (Integral Constant)
  const uint8_t kd = 0; // Tune (Derivative Constant)

  uint16_t startTime = millis();
  uint16_t leftToFReading = this->tofLeft.getDist(); 
  uint16_t rightToFReading = this->tofRight.getDist();
  uint16_t endTime = millis();

  uint16_t dt = endTime - startTime;
  error = leftToFReading - rightToFReading;

  float proportional = kp * error;
  float integral = prevIterm + (ki * (error + prevError) * dt);  
  float derivative = (kd*  (error + prevError) / dt);

  prevError = error;

  float pidTerm = proportional + integral + derivative;

  uint8_t leftMotorSpeedPid = this->motorRight.getSpeed() + (uint8_t) pidTerm;
  uint8_t rightMotorSpeedPid = this->motorLeft.getSpeed() - (uint8_t) pidTerm;

  if (leftMotorSpeedPid > this->maxDriveSpeed) { // Tune
    leftMotorSpeedPid = this->maxDriveSpeed;
  }
  if (rightMotorSpeedPid > this->maxDriveSpeed)
  {
    rightMotorSpeedPid = this->maxDriveSpeed;
  }

  // Returns the turnForward or turnBackward Speed for Motors ////////
  this->motorLeft.turnForward(leftMotorSpeedPid);
  this->motorRight.turnForward(rightMotorSpeedPid);
  ////////////////////////////////////////////////////////////////////

}